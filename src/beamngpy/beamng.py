"""
.. module:: beamng
    :platform: Windows
    :synopsis: Contains the main :py:class:`.BeamNGPy` class used to interface
               with BeamNG.drive.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
"""

import base64
import logging as log
import mmap
import numpy as np
import os
import queue
import shutil
import signal
import socket
import subprocess
import sys
import time
import zipfile
import warnings

from pathlib import Path
from threading import Thread
from time import sleep

import msgpack

from PIL import Image

from .scenario import ScenarioObject

from .beamngcommon import ack, send_msg, recv_msg, ENV, BNGError
from .beamngcommon import BNGValueError, angle_to_quat
from .beamngcommon import raise_rot_deprecation_warning

PROTOCOL_VERSION = 'v1.17'

BINARIES = [
    'Bin64/BeamNG.drive.x64.exe',
    'Bin64/BeamNG.research.x64.exe',
]


def log_exception(extype, value, trace):
    """
    Hook to log uncaught exceptions to the logging framework. Register this as
    the excepthook with `sys.excepthook = log_exception`.
    """
    log.exception("Uncaught exception: ", exc_info=(extype, value, trace))


def setup_logging(log_file=None, activateWarnings=True):
    """
    Sets up the logging framework to log to the given log_file and to STDOUT.
    If the path to the log_file does not exist, directories for it will be
    created.
    """
    handlers = []
    if log_file:
        if os.path.exists(log_file):
            backup = '{}.1'.format(log_file)
            shutil.move(log_file, backup)
        file_handler = log.FileHandler(log_file, 'w', 'utf-8')
        handlers.append(file_handler)

    term_handler = log.StreamHandler()
    handlers.append(term_handler)
    fmt = '%(asctime)s %(levelname)-8s %(message)s'
    log.basicConfig(handlers=handlers, format=fmt, level=log.DEBUG)

    sys.excepthook = log_exception

    if activateWarnings:
        warnings.simplefilter('default')
        log.captureWarnings(True)

    log.info('Started BeamNGpy logging.')


def updating(fun):
    def update_wrapped(*args, **kwargs):
        update_wrapped.__doc__ = fun.__doc__
        if args[0].scenario:
            args[0].update_scenario()
        return fun(*args, **kwargs)
    return update_wrapped


class BeamNGpy:
    """
    The BeamNGpy class is the backbone of communication with the BeamNG
    simulation and offers methods of starting, stopping, and controlling the
    state of the simulator.
    """

    def __init__(self, host, port, home=None, user=None):
        """
        Instantiates a BeamNGpy instance connecting to the simulator on the
        given host and port. The home directory of the simulator can be passed
        to this constructor. If None is given, this class tries to read a
        home path from the ``BNG_HOME`` environment variable.

        Note:
            If no home path is set, this class will not work properly.

        Args:
            host (str): The host to connect to
            port (int): The port to connect to
            home (str): Path to the simulator's home directory.
            user (str): Additional optional user path to set. This path can be
                        used to set where custom files created during
                        executions will be placed if the home folder shall not
                        be touched.
        """
        self.host = host
        self.port = port
        self.next_port = self.port + 1
        self.server = None

        self.home = home
        if not self.home:
            self.home = ENV['BNG_HOME']
        if not self.home:
            raise BNGValueError('No BeamNG home folder given. Either specify '
                                'one in the constructor or define an '
                                'environment variable "BNG_HOME" that '
                                'points to where your copy of BeamNG.* is.')

        self.home = Path(self.home).resolve()
        self.binary = self.determine_binary()

        if user:
            self.user = Path(user).resolve()
        else:
            self.user = self.determine_userpath()

        self.process = None
        self.skt = None

        self.scenario = None

    def determine_userpath(self):
        """
        Tries to find the userpath based on the beamng installation if the user
        did not provide a custom userpath.
        """
        user = Path.home() / 'Documents'
        if '.research' in self.binary:
            user = user / 'BeamNG.research'
        else:
            user = user / 'BeamNG.drive'
        return user

    def determine_binary(self):
        """
        Tries to find one of the common BeamNG-binaries in the specified home
        path and returns the discovered path as a string.

        Returns:
            Path to the binary as a string.

        Raises:
            BNGError: If no binary could be determined.
        """
        choice = None
        for option in BINARIES:
            binary = self.home / option
            if binary.exists():
                choice = binary
                break

        if not choice:
            raise BNGError('No BeamNG binary found in BeamNG home. Make '
                           'sure any of these exist in the BeamNG home '
                           'folder: %s'.format(','.join(BINARIES)))

        log.debug('Determined BeamNG.* binary to be: %s', choice)
        return str(choice)

    def deploy_mod(self):
        mods = self.user / 'mods'
        if not mods.exists():
            mods.mkdir(parents=True)

        lua = Path(__file__).parent / 'lua'
        common = lua / 'researchCommunication.lua'
        ge = lua / 'researchGE.lua'
        ve = lua / 'researchVE.lua'

        common_name = 'lua/common/utils/researchCommunication.lua'
        ge_name = 'lua/ge/extensions/util/researchGE.lua'
        ve_name = 'lua/vehicle/extensions/researchVE.lua'

        with zipfile.ZipFile(str(mods / 'BeamNGpy.zip'), 'w') as ziph:
            ziph.write(common, arcname=common_name)
            ziph.write(ge, arcname=ge_name)
            ziph.write(ve, arcname=ve_name)

    def prepare_call(self, extensions, *args, **opts):
        """
        Prepares the command line call to execute to start BeamNG.*.
        according to this class' and the global configuration.

        Returns:
            List of shell components ready to be called in the
            :mod:`subprocess` module.
        """
        if extensions is None:
            extensions = []

        extensions.insert(0, 'util/researchGE')
        lua = ("registerCoreModule('{}');" * len(extensions))[:-1]
        lua = lua.format(*extensions)
        call = [
            self.binary,
            '-console',
            '-rport',
            str(self.port),
            '-rhost',
            str(self.host),
            '-nosteam',
            '-physicsfps',
            '4000',
            '-lua',
            lua,
        ]

        for arg in args:
            call.append(arg)
        for key, val in opts.items():
            call.extend([key, val])

        if self.user:
            call.append('-userpath')
            call.append(str(self.user))

        return call

    def start_beamng(self, extensions, *args, **opts):
        """
        Spawns a BeamNG.* process and retains a reference to it for later
        termination.
        """
        call = self.prepare_call(extensions, *args, **opts)
        log.debug('Starting BeamNG process: %s', call)
        self.process = subprocess.Popen(call)

    def kill_beamng(self):
        """
        Kills the running BeamNG.* process.
        """
        if not self.process:
            return

        log.debug('Killing BeamNG process...')
        if os.name == "nt":
            with open(os.devnull, 'w') as devnull:
                subprocess.call([
                    'taskkill', '/F', '/T', '/PID',
                    str(self.process.pid)], stdout=devnull,
                    stderr=devnull
                )
        else:
            os.kill(self.process.pid, signal.SIGTERM)

        self.process = None

    def start_server(self):
        """
        Binds a server socket to the configured host & port and starts
        listening on it.
        """
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.host, self.port))
        self.server.listen()
        log.info('Started BeamNGpy server on %s:%s', self.host, self.port)

    def send(self, data):
        """
        Helper method for sending data over this instance's socket.

        Args:
            data (dict): The data to send.
        """
        return send_msg(self.skt, data)

    def recv(self):
        """
        Helper method for receiving data over this instance's socket.

        Returns:
            The data received.
        """
        return recv_msg(self.skt)

    def open(self, extensions=None, *args, launch=True, **opts):
        """
        Starts a BeamNG.* process, opens a server socket, and waits for the
        spawned BeamNG.* process to connect. This method blocks until the
        process started and is ready.

        Args:
            launch (bool): Whether to launch a new process or connect to a
                           running one on the configured host/port. Defaults to
                           True.
        """
        log.info('Opening BeamNPy instance...')
        self.deploy_mod()
        self.start_server()
        if launch:
            self.start_beamng(extensions, *args, **opts)

        self.server.settimeout(300)
        self.skt, addr = self.server.accept()
        self.skt.settimeout(300)

        log.debug('Connection established. Awaiting "hello"...')
        hello = self.recv()
        assert hello['type'] == 'Hello'
        if hello['version'] != PROTOCOL_VERSION:
            print('BeamNGpy and BeamNG.* version mismatch: '
                  'BeamNGpy {}, BeamNG.* {}'.format(PROTOCOL_VERSION,
                                                    hello['version']))
            print('Make sure both this library and BeamNG.* are up to date.')
            print('Operation will proceed, but some features might not work.')

        log.info('Started BeamNGpy communicating on %s', addr)
        return self

    def close(self):
        """
        Kills the BeamNG.* process and closes the server.
        """
        log.info('Closing BeamNGpy instance...')
        if self.scenario:
            self.scenario.close()
            self.scenario = None

        self.server.close()
        self.kill_beamng()

    def hide_hud(self):
        """
        Hides the HUD in the simulator.
        """
        data = dict(type='HideHUD')
        self.send(data)

    def show_hud(self):
        """
        Shows the HUD in the simulator.
        """
        data = dict(type='ShowHUD')
        self.send(data)

    def connect_vehicle(self, vehicle, port=None):
        """
        Creates a server socket for the given vehicle and sends a connection
        request for it to the simulation. This method does not wait for the
        connection to be established but rather returns the respective server
        socket to the caller.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle instance to be connected.
            port (int): Optional. The port the vehicle should be connecting
                        over.

        Returns:
            The server socket created and waiting for a conection.
        """
        flags = vehicle.get_engine_flags()
        self.set_engine_flags(flags)

        if not port:
            port = self.next_port
            self.next_port += 1

        vehicle_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        vehicle_server.bind((self.host, port))
        vehicle_server.listen()
        log.debug('Starting vehicle server for %s on: %s:%s',
                  vehicle, self.host, port)

        connection_msg = {'type': 'VehicleConnection'}
        connection_msg['vid'] = vehicle.vid
        connection_msg['host'] = self.host
        connection_msg['port'] = port
        if vehicle.extensions is not None:
            connection_msg['exts'] = vehicle.extensions

        self.send(connection_msg)

        vehicle.connect(self, vehicle_server, port)
        return vehicle_server

    def setup_vehicles(self, scenario):
        """
        Goes over the current scenario's vehicles and establishes a connection
        between their vehicle instances and the vehicles in simulation. Engine
        flags required by the vehicles' sensor setups are sent and connect-
        hooks of the respective sensors called upon connection. This method
        blocks until all vehicles are fully connected.

        Args:
            scenario (:class:`.Scenario`): Calls functions to set up scenario
                                           objects after it has been loaded.
        """
        vehicles = scenario.vehicles
        for vehicle in vehicles.keys():
            self.connect_vehicle(vehicle)

    def load_scenario(self, scenario):
        """
        Loads the given scenario in the simulation and returns once loading
        is finished.

        Args:
            scenario (:class:`.Scenario`): The scenario to load.
        """
        info_path = scenario.get_info_path()
        info_path = info_path.replace(str(self.home), '')
        info_path = info_path.replace(str(self.user), '')
        info_path = info_path[1:]
        info_path = info_path.replace('\\', '/')
        data = {'type': 'LoadScenario', 'path': info_path}
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'MapLoaded'
        self.setup_vehicles(scenario)
        flags = scenario.get_engine_flags()
        self.set_engine_flags(flags)
        self.scenario = scenario
        self.scenario.connect(self)

    @ack('SetEngineFlags')
    def set_engine_flags(self, flags):
        """
        Sets flags in the simulation engine. Flags are given as key/value pairs
        of strings and booleans, where each string specifies a flag and the
        boolean the state to set. Possible flags are:

         * ``annotations``: Whether pixel-wise annotation should be enabled.
         * ``lidar``: Whether Lidar rendering should be enabled.

        """
        flags = dict(type='EngineFlags', flags=flags)
        self.send(flags)

    @ack('OpenedShmem')
    def open_shmem(self, name, size):
        """
        Tells the simulator to open a shared memory handle with the given
        amount of bytes.

        Args:
            name (str): The name of the shared memory to open.
            size (int): The size to map in bytes.
        """
        data = dict(type='OpenShmem', name=name, size=size)
        self.send(data)

    @ack('ClosedShmem')
    def close_shmem(self, name):
        """
        Tells the simulator to close a previously-opened shared memory handle.

        Args:
            name (str): The name of the shared memory space to close.
        """
        data = dict(type='CloseShmem', name=name)
        self.send(data)

    @ack('OpenedLidar')
    def open_lidar(self, name, vehicle, shmem, shmem_size, offset=(0, 0, 0),
                   direction=(0, -1, 0), vres=64, vangle=26.9, rps=2200000,
                   hz=20, angle=360, max_dist=120, visualized=True):
        """
        Opens a Lidar sensor instance in the simulator with the given
        parameters writing its data to the given shared memory space. The Lidar
        instance has to be assigned a unique name that is later used for
        closing.

        Args:
            name (str): The name of the Lidar instance to open. Has to be
                        unique relative to other Lidars currently opened.
            vehicle (:class:`.Vehicle`): The vehicle this Lidar is attached to.
            shmem (str): The handle of the shared memory space used to exchange
                         data.
            shmem_size (int): Size of the shared memory space that has been
                              allocated for exchange.
            offset (tuple): (X, Y, Z) coordinate triplet specifying the
                            position of the sensor relative to the vehicle's.
            direction (tuple): (X, Y, Z) coordinate triple specifying the
                               direction the Lidar is pointing towards.
            vres (int): Vertical resolution, i.e. how many lines are sampled
                        vertically.
            vangle (float): The vertical angle, i.e. how many degrees up and
                            down points are scattered.
            rps (int): The rays per second shot by the sensor.
            hz (int): The refresh rate of the sensor in Hz
            angle (float): The horizontal degrees covered, i.e. 360 degrees
                           covers the entire surroundings of the vehicle.
            max_dist (float): Maximum distance of points. Any dot farther away
                              will not show up in the sample.
            visualized (bool): Whether or not to render the Lidar sensor's
                              points in the simulator.
        """
        data = dict(type='OpenLidar')
        data['name'] = name
        data['shmem'] = shmem
        data['size'] = shmem_size
        data['vid'] = vehicle.vid
        data['offset'] = offset
        data['direction'] = direction
        data['vRes'] = vres
        data['vAngle'] = vangle
        data['rps'] = rps
        data['hz'] = hz
        data['angle'] = angle
        data['maxDist'] = max_dist
        data['visualized'] = visualized
        self.send(data)

    @ack('ClosedLidar')
    def close_lidar(self, name):
        """
        Closes the Lidar instance of the given name in the simulator.

        Args:
            name (str): The name of the Lidar instance to close.
        """
        data = dict(type='CloseLidar')
        data['name'] = name
        self.send(data)

    @ack('Teleported')
    def teleport_vehicle(self, vehicle, pos, rot=None, rot_quat=None):
        """
        Teleports the given vehicle to the given position with the given
        rotation.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to teleport.
            pos (tuple): The target position as an (x,y,z) tuple containing
                         world-space coordinates.
            rot (tuple): Optional tuple specifying rotations around the (x,y,z)
                         axes in degrees.
            rot_quat (tuple): Optional tuple (x, y, z, w) specifying vehicle
                              rotation as quaternion

        Notes:
            In the current implementation, if both ``pos`` and ``rot`` are
            specified, the vehicle will be repaired to its initial state during
            teleport.
        """
        data = dict(type='Teleport')
        data['vehicle'] = vehicle.vid
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        elif rot:
            raise_rot_deprecation_warning()
            data['rot'] = angle_to_quat(rot)
        self.send(data)

    @ack('ScenarioObjectTeleported')
    def teleport_scenario_object(self, scenario_object, pos,
                                 rot=None, rot_quat=None):
        """
        Teleports the given scenario object to the given position with the
        given rotation.

        Args:
            scenario_object (:class:`.ScenarioObject`): The vehicle to
                                                        teleport.
            pos (tuple): The target position as an (x,y,z) tuple containing
                         world-space coordinates.
            rot (tuple): Optional tuple specifying rotations around the (x,y,z)
                         axes in degrees.
            rot_quat (tuple): Optional tuple specifying object rotation as a
                              quaternion
        """
        data = dict(type='TeleportScenarioObject')
        data['id'] = scenario_object.id
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        elif rot:
            raise_rot_deprecation_warning()
            data['rot'] = angle_to_quat(rot)
        self.send(data)

    @ack('ScenarioStarted')
    def start_scenario(self):
        """
        Starts the scenario; equivalent to clicking the "Start" button in the
        game after loading a scenario. This method blocks until the countdown
        to the scenario's start has finished.
        """
        data = dict(type="StartScenario")
        self.send(data)

    @ack('ScenarioRestarted')
    def restart_scenario(self):
        """
        Restarts a running scenario.
        """
        if not self.scenario:
            raise BNGError('Need to have a scenario loaded to restart it.')

        self.scenario.restart()

        data = dict(type='RestartScenario')
        self.send(data)

    @ack('ScenarioStopped')
    def stop_scenario(self):
        """
        Stops a running scenario and returns to the main menu.
        """
        if not self.scenario:
            raise BNGError('Need to have a scenario loaded to stop it.')

        self.scenario.close()
        self.scenario = None

        data = dict(type='StopScenario')
        self.send(data)

    @ack('SetPhysicsDeterministic')
    def set_deterministic(self):
        """
        Sets the simulator to run in deterministic mode. For this to function
        properly, an amount of steps per second needs to have been specified
        in the simulator's settings, or through
        :meth:`~.BeamnGpy.set_steps_per_second`.
        """
        data = dict(type='SetPhysicsDeterministic')
        self.send(data)

    @ack('SetPhysicsNonDeterministic')
    def set_nondeterministic(self):
        """
        Disables the deterministic mode of the simulator. Any steps per second
        setting is retained.
        """
        data = dict(type='SetPhysicsNonDeterministic')
        self.send(data)

    @ack('SetFPSLimit')
    def set_steps_per_second(self, sps):
        """
        Specifies the temporal resolution of the simulation. The setting can be
        understood to determine into how many steps the simulation divides one
        second of simulation. A setting of two, for example, would mean one
        second is simulated in two steps. Conversely, to simulate one second,
        one needs to advance the simulation two steps.

        Args:
            sps (int): The steps per second to set.
        """
        data = dict(type='FPSLimit', fps=sps)
        self.send(data)

    @ack('RemovedFPSLimit')
    def remove_step_limit(self):
        """
        Removes the steps-per-second setting, making the simulation run at
        undefined time slices.
        """
        data = dict(type='RemoveFPSLimit')
        self.send(data)

    def step(self, count, wait=True):
        """
        Advances the simulation the given amount of steps, assuming it is
        currently paused. If the wait flag is set, this method blocks until
        the simulator has finished simulating the desired amount of steps. If
        not, this method resumes immediatly. This can be used to queue commands
        that should be executed right after the steps have been simulated.

        Args:
            count (int): The amount of steps to simulate.
            wait (bool): Optional. Whether to wait for the steps to be
                         simulated. Defaults to True.

        Raises:
            BNGError: If the wait flag is set but the simulator doesn't respond
                      appropriately.
        """
        data = dict(type='Step', count=count)
        data['ack'] = wait
        self.send(data)
        if wait:
            resp = self.recv()
            if resp['type'] != 'Stepped':
                raise BNGError('Wrong ACK: {} != {}'.format('Stepped',
                                                            resp['type']))

    @ack('Paused')
    def pause(self):
        """
        Sends a pause request to BeamNG.*, blocking until the simulation is
        paused.
        """
        data = dict(type='Pause')
        self.send(data)

    @ack('Resumed')
    def resume(self):
        """
        Sends a resume request to BeamNG.*, blocking until the simulation
        is resumed.
        """
        data = dict(type='Resume')
        self.send(data)

    def poll_sensors(self, vehicle):
        """
        This member function is deprecated and will be removed in future versions.
        Use 'Vehicle.poll_sensors' instead.
        Retrieves sensor values for the sensors attached to the given vehicle.
        This method correctly splits requests meant for the game engine and
        requests meant for the vehicle, sending them to their supposed
        destinations and waiting for results from them. Results from either are
        merged into one dictionary for ease of use. The received data is
        decoded by each sensor and returned, but also stored in the vehicle's
        sensor cache to avoid repeated requests.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle whose sensors are polled.

        Returns:
            The decoded sensor data from both engine and vehicle as one
            dictionary having a key-value pair for each sensor's name and the
            data received for it.
        """
        warnings.warn("'BeamNGpy.poll_sensors' is deprecated\nuse 'Vehicle.poll_sensors' instead\nthis function is going to be removed in the future", DeprecationWarning)

        vehicle.poll_sensors()
        return vehicle.sensor_cache

    def render_cameras(self):
        """
        Renders all cameras associated with the loaded scenario. These cameras
        work exactly like the ones attached to vehicles as sensors, except
        scenario cameras do not follow the vehicle they are attached to and can
        be used to get a view from the perspective of something like a
        surveillance camera, for example.

        A scenario needs to be loaded for this method to work.

        Returns:
            The rendered data for all cameras in the loaded scenario as a
            dict mapping camera name to render results.
        """
        if not self.scenario:
            raise BNGError('Need to be in a started scenario to render its '
                           'cameras.')

        engine_reqs = self.scenario.encode_requests()
        self.send(engine_reqs)
        response = self.recv()
        assert response['type'] == 'SensorData'
        camera_data = response['data']
        result = self.scenario.decode_frames(camera_data)
        return result

    def get_roads(self):
        """
        Retrieves the vertex data of all DecalRoads in the current scenario.
        The vertex data of a DecalRoad is formatted as point triples, where
        each triplet represents the left, centre, and right points of the edges
        that make up a DecalRoad.

        Returns:
            A dict mapping DecalRoad IDs to lists of point triples.
        """
        if not self.scenario:
            raise BNGError('Need to be in a started scenario to get its '
                           'DecalRoad data.')

        data = dict(type='GetDecalRoadData')
        self.send(data)
        response = self.recv()
        assert response['type'] == 'DecalRoadData'
        return response['data']

    def get_road_edges(self, road):
        """
        Retrieves the edges of the road with the given name and returns them
        as a list of point triplets. Roads are defined by a series of lines
        that specify the leftmost, center, and rightmost point in the road.
        These lines go horizontally across the road and the series of leftmost
        points make up the left edge of the road, the series of rightmost
        points make up the right edge of the road, and the series of center
        points the middle line of the road.

        Args:
            road (str): Name of the road to get edges from.

        Returns:
            The road edges as a list of (left, center, right) point triplets.
            Each point is an (X, Y, Z) coordinate triplet.
        """
        data = dict(type='GetDecalRoadEdges')
        data['road'] = road
        self.send(data)
        response = self.recv()
        assert response['type'] == 'DecalRoadEdges'
        return response['edges']

    def get_gamestate(self):
        """
        Retrieves the current game state of the simulator. The game state is
        returned as a dictionary containing a ``state`` entry that is either:

            * ``scenario`` when a scenario is loaded
            * ``menu`` otherwise

        If a scenario is loaded, the resulting dictionary also contains a
        ``scenario_state`` entry whose value is ``pre-running`` if the scenario
        is currently at the start screen or ``running`` otherwise.

        Returns:
            The game state as a dictionary as described above.
        """
        data = dict(type='GameStateRequest')
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'GameState'
        return resp

    @ack('TimeOfDayChanged')
    def set_tod(self, tod):
        """
        Sets the current time of day. The time of day value is given as a float
        between 0 and 1. How this value affects the lighting of the scene is
        dependant on the map's TimeOfDay object.

        Args:
            tod (float): Time of day beteen 0 and 1.
        """
        data = dict(type='TimeOfDayChange')
        data['tod'] = tod
        self.send(data)

    @ack('WeatherPresetChanged')
    def set_weather_preset(self, preset, time=1):
        """
        Triggers a change to a different weather preset. Weather presets affect
        multiple settings at once (time of day, wind speed, cloud coverage,
        etc.) and need to have been defined first. Example json objects
        defining weather presets can be found in BeamNG.research's
        ``art/weather/defaults.json`` file.

        Args:
            preset (str): The name of the preset to switch to. Needs to be
                          defined already within the simulation.
            time (float): Time in seconds the transition from the current
                          settings to the preset's should take.
        """
        data = dict(type='SetWeatherPreset')
        data['preset'] = preset
        data['time'] = time
        self.send(data)

    def await_vehicle_spawn(self, vid):
        """
        Waits for the vehicle with the given name to spawn and returns once it
        has.

        Args:
            vid (str): The name of the  vehicle to wait for.
        """
        req = dict(type='WaitForSpawn')
        req['name'] = vid
        self.send(req)
        resp = self.recv()
        assert resp['type'] == 'VehicleSpawned'
        assert resp['name'] == vid

    def update_scenario(self):
        """
        Updates the :attr:`.Vehicle.state` field of each vehicle in the
        currently running scenario.
        """
        if not self.scenario:
            raise BNGError('Need to have a senario loaded to update it.')

        data = dict(type='UpdateScenario')
        data['vehicles'] = list()
        for vehicle in self.scenario.vehicles.keys():
            data['vehicles'].append(vehicle.vid)
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'ScenarioUpdate'
        for name, vehicle_state in resp['vehicles'].items():
            vehicle = self.scenario.get_vehicle(name)
            if vehicle:
                vehicle.state = vehicle_state

    @ack('GuiMessageDisplayed')
    def display_gui_message(self, msg):
        """
        Displays a toast message in the user interface of the simulator.

        Args:
            msg (str): The message to display.
        """
        data = dict(type='DisplayGuiMessage')
        data['message'] = msg
        self.send(data)

    @ack('VehicleSwitched')
    def switch_vehicle(self, vehicle):
        """
        Switches to the given :class:`.Vehicle`. This means that the
        simulator's main camera, inputs by the user, and so on will all focus
        on that vehicle from now on.

        Args:
            vehicle (:class:`.Vehicle`): The target vehicle.
        """
        data = dict(type='SwitchVehicle')
        data['vid'] = vehicle.vid
        self.send(data)

    @ack('FreeCameraSet')
    def set_free_camera(self, pos, direction):
        """
        Sets the position and direction of the free camera. The free camera is
        one that does not follow any particular vehicle, but can instead be
        put at any spot and any position on the map.

        Args:
            pos (tuple): The position of the camera as a (x, y, z) triplet.
            direction (tuple): The directional vector of the camera as a
                               (x, y, z) triplet.
        """
        data = dict(type='SetFreeCamera')
        data['pos'] = pos
        data['dir'] = direction
        self.send(data)

    @ack('ParticlesSet')
    def set_particles_enabled(self, enabled):
        """
        En-/disabled visual particle emmission.

        Args:
            enabled (bool): Whether or not to en- or disabled effects.
        """
        data = dict(type='ParticlesEnabled')
        data['enabled'] = enabled
        self.send(data)

    @ack('PartsAnnotated')
    def annotate_parts(self, vehicle):
        """
        Triggers per-part annotation for the given :class:`.Vehicle`.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to annotate.
        """
        data = dict(type='AnnotateParts')
        data['vid'] = vehicle.vid
        self.send(data)

    @ack('AnnotationsReverted')
    def revert_annotations(self, vehicle):
        """
        Reverts the given vehicle's annotations back to the object-based mode,
        removing the per-part annotations.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to annotate.
        """
        data = dict(type='RevertAnnotations')
        data['vid'] = vehicle.vid
        self.send(data)

    def get_part_annotations(self, vehicle):
        data = dict(type='GetPartAnnotations')
        data['vid'] = vehicle.vid
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'PartAnnotations'
        return resp['colors']

    def get_part_annotation(self, part):
        data = dict(type='GetPartAnnotation')
        data['part'] = part
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'PartAnnotation'
        if 'color' in resp:
            return resp['color']
        return None

    def get_scenario_name(self):
        """
        Retrieves the name of the currently-loaded scenario in the simulator.

        Returns:
            The name of the loaded scenario as a string.
        """
        data = dict(type='GetScenarioName')
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'ScenarioName'
        return resp['name']

    def spawn_vehicle(self, vehicle, pos, rot,
                      rot_quat=(0, 0, 0, 1), cling=True):
        """
        Spawns the given :class:`.Vehicle` instance in the simulator. This
        method is meant for spawning vehicles *during the simulation*. Vehicles
        that are known to be required before running the simulation should be
        added during scenario creation instead.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to be spawned.
            pos (tuple): Where to spawn the vehicle as a (x, y, z) triplet.
            rot (tuple): The rotation of the vehicle as a triplet of Euler
                         angles.
            rot_quat (tuple): Vehicle rotation in form of a quaternion
            cling (bool): If set, the z-coordinate of the vehicle's position
                          will be set to the ground level at the given
                          position to avoid spawning the vehicle below ground
                          or in the air.
        """
        data = dict(type='SpawnVehicle', cling=cling)
        data['name'] = vehicle.vid
        data['model'] = vehicle.options['model']
        data['pos'] = pos
        if rot:
            raise_rot_deprecation_warning()
            rot_quat = angle_to_quat(rot)
        data['rot'] = rot_quat
        data.update(vehicle.options)
        self.send(data)
        resp = self.recv()
        self.connect_vehicle(vehicle)
        assert resp['type'] == 'VehicleSpawned'

    def despawn_vehicle(self, vehicle):
        """
        Despawns the given :class:`.Vehicle` from the simulation.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to despawn.
        """
        vehicle.disconnect()
        data = dict(type='DespawnVehicle')
        data['vid'] = vehicle.vid
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'VehicleDespawned'

    def find_objects_class(self, clazz):
        """
        Scans the current environment in the simulator for objects of a
        certain class and returns them as a list of :class:`.ScenarioObject`.

        What kind of classes correspond to what kind of objects is described
        in the BeamNG.drive documentation.

        Args:
            clazz (str): The class name of objects to find.

        Returns:
            Found objects as a list.
        """
        data = dict(type='FindObjectsClass')
        data['class'] = clazz
        self.send(data)
        resp = self.recv()
        ret = list()
        for obj in resp['objects']:
            sobj = ScenarioObject(obj['id'], obj['name'], obj['type'],
                                  tuple(obj['position']),
                                  None,
                                  tuple(obj['scale']),
                                  rot_quat=tuple(obj['rotation']),
                                  **obj['options'])
            ret.append(sobj)

        return ret

    @ack('CreatedCylinder')
    def create_cylinder(self, name, radius, height, pos, rot,
                        rot_quat=None, material=None):
        """
        Creates a procedurally generated cylinder mesh with the given
        radius and height at the given position and rotation. The material
        can optionally be specified and a name can be assigned for later
        identification.

        Args:
            name (str): Name for the mesh. Should be unique.
            radius (float): The radius of the cylinder's base circle.
            height (float): The between top and bottom circles of the
                            cylinder.
            pos (tuple): (X, Y, Z) coordinate triplet specifying the cylinder's
                         position.
            rot (tuple): Triplet of Euler angles specifying rotations around
                         the (X, Y, Z) axes.
            rot_quat (tuple): Quaternion specifying the cylinder's rotation
            material (str): Optional material name to use as a texture for the
                            mesh.
        """
        data = dict(type='CreateCylinder')
        data['radius'] = radius
        data['height'] = height
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        else:
            raise_rot_deprecation_warning()
            data['rot'] = angle_to_quat(rot)
        data['name'] = name
        data['material'] = material
        self.send(data)

    @ack('CreatedBump')
    def create_bump(self, name, width, length, height, upper_length,
                    upper_width, pos, rot, rot_quat=None, material=None):
        """
        Creates a procedurally generated bump with the given properties at the
        given position and rotation. The material can optionally be specified
        and a name can be assigned for later identification.

        Args:
            name (str): Name for the mesh. Should be unique.
            width (float): The width of the bump, i.e. its size between left
                           and right edges.
            length (float): The length of the bump, i.e. the distances from
                            up and downward slopes.
            height (float): The height of the tip.
            upper_length (float): The length of the tip.
            upper_width (float): The width of the tip.
            pos (tuple): (X, Y, Z) coordinate triplet specifying the cylinder's
                         position.
            rot (tuple): Triplet of Euler angles specifying rotations around
                         the (X, Y, Z) axes.
            rot_quat (tuple): Quaternion specifying the bump's rotation
            material (str): Optional material name to use as a texture for the
                            mesh.
        """
        data = dict(type='CreateBump')
        data['width'] = width
        data['length'] = length
        data['height'] = height
        data['upperLength'] = upper_length
        data['upperWidth'] = upper_width
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        else:
            raise_rot_deprecation_warning()
            data['rot'] = angle_to_quat(rot)
        data['name'] = name
        data['material'] = material
        self.send(data)

    @ack('CreatedCone')
    def create_cone(self, name, radius, height, pos, rot, rot_quat=None,
                    material=None):
        """
        Creates a procedurally generated cone with the given properties at the
        given position and rotation. The material can optionally be specified
        and a name can be assigned for later identification.

        Args:
            name (str): Name for the mesh. Should be unique.
            radius (float): Radius of the base circle.
            height (float): Distance of the tip to the base circle.
            pos (tuple): (X, Y, Z) coordinate triplet specifying the cylinder's
                         position.
            rot (tuple): Triplet of Euler angles specifying rotations around
                         the (X, Y, Z) axes.
            rot_quat (tuple): Quaternion specifying the cone's rotation
            material (str): Optional material name to use as a texture for the
                            mesh.
        """
        data = dict(type='CreateCone')
        data['radius'] = radius
        data['height'] = height
        data['material'] = material
        data['name'] = name
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        else:
            raise_rot_deprecation_warning()
            data['rot'] = angle_to_quat(rot)
        self.send(data)

    @ack('CreatedCube')
    def create_cube(self, name, size, pos, rot, rot_quat=None, material=None):
        """
        Creates a procedurally generated cube with the given properties at the
        given position and rotation. The material can optionally be specified
        and a name can be assigned for later identification.

        Args:
            name (str): Name for the mesh. Should be unique.
            size (tuple): A triplet specifying the (length, width, height) of
                          the cuboid.
            pos (tuple): (X, Y, Z) coordinate triplet specifying the cylinder's
                         position.
            rot (tuple): Triplet of Euler angles specifying rotations around
                         the (X, Y, Z) axes.
            rot_quat (tuple): Quaternion specifying the cube's rotation
            material (str): Optional material name to use as a texture for the
                            mesh.
        """
        data = dict(type='CreateCube')
        data['size'] = size
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        else:
            raise_rot_deprecation_warning()
            data['rot'] = angle_to_quat(rot)
        data['material'] = material
        data['name'] = name
        self.send(data)

    @ack('CreatedRing')
    def create_ring(self, name, radius, thickness, pos, rot, rot_quat=None,
                    material=None):
        """
        Creates a procedurally generated ring with the given properties at the
        given position and rotation. The material can optionally be specified
        and a name can be assigned for later identification.

        Args:
            name (str): Name for the mesh. Should be unique.
            radius (float): Radius of the circle encompassing the ring.
            thickness (float): Thickness of the rim.
            pos (tuple): (X, Y, Z) coordinate triplet specifying the cylinder's
                         position.
            rot (tuple): Triplet of Euler angles specifying rotations around
                         the (X, Y, Z) axes.
            rot_quat (tuple): Quaternion specifying the ring's rotation
            material (str): Optional material name to use as a texture for the
                            mesh.
        """
        data = dict(type='CreateRing')
        data['radius'] = radius
        data['thickness'] = thickness
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        else:
            raise_rot_deprecation_warning()
            data['rot'] = angle_to_quat(rot)
        data['material'] = material
        data['name'] = name
        self.send(data)

    def get_vehicle_bbox(self, vehicle):
        """
        Retrieves the current bounding box of the vehicle. The bounding box
        corresponds to the vehicle's location/rotation in world space, i.e. if
        the vehicle moves/turns, the bounding box moves acoordingly. Note that
        the bounding box contains the min/max coordinates of the entire
        vehicle. This means that the vehicle losing a part like a mirror will
        cause the bounding box to "expand" while the vehicle moves as the
        mirror is left behind, but still counts as part of the box containing
        the vehicle.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to get the bounding box of

        Returns:
            The vehicle's current bounding box as a dictionary of eight points.
            Points are named following the convention that the cuboid has a
            "near" rectangle towards the rear of the vehicle and "far"
            rectangle towards the front. The points are then named like this:

            * `front_bottom_left`: Bottom left point of the front rectangle as
                                   an (x, y ,z) triplet
            * `front_bottom_right`: Bottom right point of the front rectangle
                                    as an (x, y, z) triplet
            * `front_top_left`: Top left point of the front rectangle as an
                                (x, y, z) triplet
            * `front_top_right`: Top right point of the front rectangle as an
                                 (x, y, z) triplet
            * `rear_bottom_left`: Bottom left point of the rear rectangle as an
                                  (x, y, z) triplet
            * `rear_bottom_right`: Bottom right point of the rear rectangle as
                                   an (x, y, z) triplet
            * `rear_top_left`: Top left point of the rear rectangle as an
                               (x, y, z) triplet
            * `rear_top_right`: Top right point of the rear rectangle as an
                                (x, y, z) triplet
        """
        data = dict(type='GetBBoxCorners')
        data['vid'] = vehicle.vid
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'BBoxCorners'
        points = resp['points']
        bbox = {
            'front_bottom_left': points[3],
            'front_bottom_right': points[0],
            'front_top_left': points[2],
            'front_top_right': points[1],
            'rear_bottom_left': points[7],
            'rear_bottom_right': points[4],
            'rear_top_left': points[6],
            'rear_top_right': points[5],
        }
        return bbox

    @ack('GravitySet')
    def set_gravity(self, gravity=-9.807):
        """
        Sets the strength of gravity in the simulator.

        Args:
            gravity (float): The gravity value to set. The default one is
                             that of earth (-9.807)
        """
        data = dict(type='SetGravity')
        data['gravity'] = gravity
        self.send(data)

    def get_available_vehicles(self):
        """
        Retrieves a dictionary of vehicles known to the simulator that map
        to various properties of the vehicle and a list of pre-configured
        vehicle configurations.

        Returns:
            A mapping of model names to vehicle properties & configs.

        Raises:
            BNGError: If the game is not running to accept a request.
        """
        if not self.skt:
            raise BNGError('The game needs to be started to retrieve '
                           'vehicles.')

        data = dict(type='GetAvailableVehicles')
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'AvailableVehicles'
        return resp

    @ack('TrafficStarted')
    def start_traffic(self, participants):
        """
        Enables traffic simulation for the given list of vehicles.

        Args:
            participants (list): List of vehicles that will be part of the
                                 simulation. These vehicles need to be spawned
                                 beforehand and the simulation will take
                                 control of them.
        """
        participants = [p.vid for p in participants]
        data = dict(type='StartTraffic')
        data['participants'] = participants
        self.send(data)

    @ack('TrafficStopped')
    def stop_traffic(self, stop=False):
        """
        Stops the traffic simulation.

        Args:
            stop (bool): Whether or not to stop the vehicles participating in
                         traffic. If True, vehicles will come to a halt, if
                         False, the AI will simply stop controlling the
                         vehicle.
        """
        data = dict(type='StopTraffic')
        data['stop'] = stop
        self.send(data)

    @ack('SettingsChanged')
    def change_setting(self, key, value):
        """
        Changes a setting in the game. Examples of the key and value pairs
        given to this method can be found in your game's settings ini files.
        These are usually in <userpath>/settings/game-settings.ini or
        <userpath>/settings/cloud/game-settings-cloud.ini.

        Args:
            key (str): The key of the setting that is to be changed
            value (str): The desired value.
        """
        data = dict(type='ChangeSetting')
        data['key'] = key
        data['value'] = value
        self.send(data)

    @ack('GraphicsSettingApplied')
    def apply_graphics_setting(self):
        """
        Makes the game apply a graphics setting that has been changed since
        startup or the last time settings were applied. A call to this is
        required after changing settings like whether or not the game is
        in fullscreen or the resolution, otherwise those settings will only
        take effect after the next launch.
        """
        data = dict(type='ApplyGraphicsSetting')
        self.send(data)

    @ack('ExecutedLuaChunkGE')
    def queue_lua_command(self, chunk):
        """
        Executes one lua chunk in the game engine VM.

        Args:
            chunk(str): lua chunk as a string
        """
        data = dict(type='QueueLuaCommandGE')
        data['chunk'] = chunk
        self.send(data)

    @ack('RelativeCamSet')
    def set_relative_camera(self, pos, rot=None, rot_quat=None):
        """
        Switches the camera mode for the currently-entered vehicle to the
        'relative' mode in which the camera can be placed at an arbitrary point
        relative to the vehicle, moving along with it as it drives around.

        Args:
            pos (tuple): (x, y, z) tuple of the camera's position relative to
                         the vehicle.
            rot (tuple): Euler angles expressing the rotation of the camera.
            rot_quat (tuple): The camera's rotation but written as a quat.
        """
        if rot_quat is None:
            if rot is not None:
                rot_quat = angle_to_quat(rot)
        data = dict(type='SetRelativeCam')
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        self.send(data)

    def add_debug_line(self, points, point_colors,
                       spheres=None, sphere_colors=None,
                       cling=False, offset=0):
        """
        Function use is deprecated, use 'add_debug_polyline' instead!
        Adds a visual line to be rendered by BeamNG. This is mainly used for
        debugging purposes, but can also be used to add visual indicators to
        the user. A line is given as a series of points encoded as (x, y, z)
        triplets and also a list of colors given as (r, g, b, a) quartets.
        Additionally, it's possible to give a list of spheres to be rendered
        alongside the line; spheres are specified similar to the line: a list
        of (x, y, z, r) points, where r is the radius of the sphere. A list of
        colors that the respective spheres should have is also given as a list
        of (r, g, b, a) quartets.

        Args:
            points (list): List of points in the line given as (x, y, z)
                           coordinate triplets.
            point_colors (list): List of colors as (r, g, b, a) quartets, each
                                 value expressing red, green, blue, and alpha
                                 intensity from 0 to 1. Only the first entry is used.
            spheres (list): Optional list of points where spheres should be
                            rendered, given as (x, y, z, r) tuples where x,y,z
                            are coordinates and r the radius of the sphere.
            sphere_colors (list): Optional list of sphere colors that contains
                                  the desired color of spheres in the sphere
                                  list as a (r, g, b, a) quartet for each
                                  sphere.
            cling (bool): Whether or not to align the given coordinates to the
                          ground, e.g. set all z-coords to the ground height
                          below the given x, y coords.
            offset (float): A height offset that is used alongside the cling
                            flag. This is value is added to computed z-coords
                            and can be used to make the line float above the
                            ground by, for example, adding 10cm to the z value.

        Returns:
            The ID of the added debug line that can be used to remove the line
        """
        warnings.warn('use of "add_debug_line" deprecated it will be removed in future versions, use "add_debug_polyline" and "add_debug_spheres" instead')
        if spheres:
            coordinates = [s[:3] for s in spheres]
            radii = [s[3] for s in spheres]
            self.add_debug_spheres(coordinates, radii, sphere_colors, cling, offset)

        lineID = self.add_debug_polyline(points, point_colors[0], cling, offset)
        return lineID

    def remove_debug_line(self, line_id):
        warnings.warn('use of "remove_debug_line" deprecated it will be removed in future versions, use "add_debug_polyline" instead')
        self.remove_debug_polyline(line_id)

    def add_debug_spheres(self, coordinates, radii, rgba_colors, cling=False, offset=0):
        data = dict(type="AddDebugSpheres")
        assert len(coordinates) == len(radii) == len(rgba_colors)
        data['coordinates'] = coordinates
        data['radii'] = radii
        data['colors'] = rgba_colors
        data['cling'] = cling
        data['offset'] = offset
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'DebugSphereAdded'
        return resp['sphereIDs']

    @ack('DebugObjectsRemoved')
    def remove_debug_spheres(self, sphere_ids):
        data = dict(type='RemoveDebugObjects')
        data['objType'] = 'spheres'
        data['objIDs'] = sphere_ids
        self.send(data)
    
    def add_debug_polyline(self, coordinates, rgba_color, cling=False, offset=0):
        data = dict(type='AddDebugPolyline')
        data['coordinates'] = coordinates
        data['color'] = rgba_color
        data['cling'] = cling
        data['offset'] = offset
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'DebugPolylineAdded'
        return resp['lineID']

    @ack('DebugObjectsRemoved')
    def remove_debug_polyline(self, line_id):
        data = dict(type='RemoveDebugObjects')
        data['objType'] = 'polylines'
        data['objIDs'] = [line_id]
        self.send(data)

    def add_debug_cylinder(self, circle_positions, radius, rgba_color):
            data = dict(type='AddDebugCylinder')
            data['circlePositions'] = circle_positions
            data['radius'] = radius
            data['color'] = rgba_color
            self.send(data)
            resp = self.recv()
            assert resp['type'] == 'DebugCylinderAdded'
            return resp['cylinderID']

    @ack('DebugObjectsRemoved')
    def remove_debug_cylinder(self, cylinder_id):
        data = dict(type='RemoveDebugObjects')
        data['objType'] = 'cylinders'
        data['objIDs'] = [cylinder_id]
        self.send(data)

    def add_debug_triangle(self, vertices, rgba_color, cling=False, offset=0):
            data = dict(type='AddDebugTriangle')
            data['vertices'] = vertices
            data['color'] = rgba_color
            data['cling'] = cling
            data['offset'] = offset
            self.send(data)
            resp = self.recv()
            assert resp['type'] == 'DebugTriangleAdded'
            return resp['triangleID']

    @ack('DebugObjectsRemoved')
    def remove_debug_triangle(self, triangle_id):
        data = dict(type='RemoveDebugObjects')
        data['objType'] = 'triangles'
        data['objIDs'] = [triangle_id]
        self.send(data)

    def add_debug_rectangle(self, vertices, rgba_color, cling=False, offset=0):
            data = dict(type='AddDebugRectangle')
            data['vertices'] = vertices
            data['color'] = rgba_color
            data['cling'] = cling
            data['offset'] = offset
            self.send(data)
            resp = self.recv()
            assert resp['type'] == 'DebugRectangleAdded'
            return resp['rectangleID']

    @ack('DebugObjectsRemoved')
    def remove_debug_rectangle(self, rectangle_id):
        data = dict(type='RemoveDebugObjects')
        data['objType'] = 'rectangles'
        data['objIDs'] = [rectangle_id]
        self.send(data)

    def add_debug_text(self, origin, content, rgba_color, cling=False, offset=0):
            data = dict(type='AddDebugText')
            data['origin'] = origin
            data['content'] = content
            data['color'] = rgba_color
            data['cling'] = cling
            data['offset'] = offset
            self.send(data)
            resp = self.recv()
            assert resp['type'] == 'DebugTextAdded'
            return resp['textID']

    @ack('DebugObjectsRemoved')
    def remove_debug_text(self, text_id):
        data = dict(type='RemoveDebugObjects')
        data['objType'] = 'text'
        data['objIDs'] = [text_id]
        self.send(data)

    def add_debug_square_prism(self, end_points, end_point_dims, rgba_color):
            data = dict(type='AddDebugSquarePrism')
            data['endPoints'] = end_points
            data['dims'] = end_point_dims
            data['color'] = rgba_color
            self.send(data)
            resp = self.recv()
            assert resp['type'] == 'DebugSquarePrismAdded'
            return resp['prismID']

    @ack('DebugObjectsRemoved')
    def remove_debug_square_prism(self, prism_id):
        data = dict(type='RemoveDebugObjects')
        data['objType'] = 'squarePrisms'
        data['objIDs'] = [prism_id]
        self.send(data)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
