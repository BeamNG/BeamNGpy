"""
.. module:: beamng
    :platform: Windows
    :synopsis: Contains the main :py:class:`.BeamNGPy` class used to interface
               with BeamNG.drive.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
.. moduleauthor:: Dave Stark <dstark@beamng.gmbh>
.. moduleauthor:: Adam Ivora <aivora@beamng.gmbh>
"""

import os
import platform
import signal
import socket
import subprocess

from pathlib import Path
from time import sleep
import logging

from .level import Level
from .scenario import Scenario, ScenarioObject
from .vehicle import Vehicle

from .beamngcommon import send_msg, recv_msg
from .beamngcommon import angle_to_quat
from .beamngcommon import ack
from .beamngcommon import BNGError, BNGValueError
from .beamngcommon import PROTOCOL_VERSION, ENV
from .beamngcommon import set_up_simple_logging, LOGGER_ID, create_warning

BINARIES = [
    'Bin64/BeamNG.tech.x64.exe',
    'Bin64/BeamNG.drive.x64.exe',
]
BINARIES_LINUX = [
    'BinLinux/BeamNG.tech.x64',
    'BinLinux/BeamNG.drive.x64'
]

RESEARCH_HELPER = 'researchHelper.txt'

module_logger = logging.getLogger(f"{LOGGER_ID}.beamng")
module_logger.setLevel(logging.DEBUG)


def log_exception(extype, value, trace):
    """
    Hook to log uncaught exceptions to the logging framework. Register this as
    the excepthook with `sys.excepthook = log_exception`.
    """
    module_logger.exception("Uncaught exception: ",
                            exc_info=(extype, value, trace))


def setup_logging(log_file=None, activateWarnings=True, level=logging.INFO):
    """
    Sets up the logging framework to log to the given log_file and to STDOUT.
    If the path to the log_file does not exist, directories for it will be
    created.

    Args:
        log_file (str): filename for log
        activateWarnings (bool): whether to redirect warnings to the logger. Beware that this modifies the warnings settings. Optional.
        level (int): log level of handler that is created for the log file, optional
    """
    set_up_simple_logging(log_file, activateWarnings, level)
    warn_msg = str('The use of `beamng.setup_logging` is deprecated and will'
                   ' be removed in future versions. '
                   'Use `beamngcommon.set_up_simple_logging` or '
                   '`beamngcommon.config_logging` instead.')
    create_warning(warn_msg, DeprecationWarning)


class BeamNGpy:
    """
    The BeamNGpy class is the backbone of communication with the BeamNG
    simulation and offers methods of starting, stopping, connecting to, and
    controlling the state of the simulator.
    """
    @staticmethod
    def read_effective_userpath(userpath):
        reseach_helper = Path(userpath) / RESEARCH_HELPER
        if reseach_helper.exists():
            with open(reseach_helper) as infile:
                return infile.read().strip()
        return None

    def __init__(self, host, port, home="C:/game", user=None, remote=False):
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
            remote (bool): Set to true when using the BeamNGpy library on a different system then BeamNG.tech
        """
        self.logger = logging.getLogger(f'{LOGGER_ID}.BeamNGpy')
        self.logger.setLevel(logging.DEBUG)
        self.host = host
        self.port = port
        self.remote = remote
        self.home = home

        if not self.remote:
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

        self.effective_user = None

        self.process = None
        self.skt = None

        self.scenario = None

    def setup_workspace(self):
        try:
            self.start_beamng(None, lua='shutdown(0)')
            self.process.wait(timeout=600)
        except Exception as err:
            self.logger.exception(err)
        finally:
            self.kill_beamng()
            self.process = None

    def determine_userpath(self):
        """
        Tries to find the userpath based on the beamng installation if the user
        did not provide a custom userpath.
        """
        user = Path.home() / 'AppData'
        user = user / 'Local'
        if '.research' in self.binary:
            user = user / 'BeamNG.research'
        elif '.tech' in self.binary:
            user = user / 'BeamNG.tech'
        else:
            user = user / 'BeamNG.drive'
        self.logger.debug(f'Userpath is set to {user.as_posix()}')
        return user

    def determine_effective_userpath(self):
        self.setup_workspace()
        effective_userpath = BeamNGpy.read_effective_userpath(self.user)
        if not effective_userpath:
            effective_userpath = BeamNGpy.read_effective_userpath(self.user)
        self.effective_user = effective_userpath
        self.logger.debug('Automatically determined effective user path for '
                          f'mod: {self.effective_user}')

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
        binaries = BINARIES_LINUX if platform.system() == 'Linux' else BINARIES
        for option in binaries:
            binary = self.home / option
            if binary.exists():
                choice = binary
                break

        if not choice:
            raise BNGError('No BeamNG binary found in BeamNG home. Make '
                           'sure any of these exist in the BeamNG home '
                           f'folder: {", ".join(binaries)}')

        self.logger.debug(f'Determined BeamNG.* binary to be: {choice}')
        return str(choice)

    def prepare_call(self, extensions, *args, **usr_opts):
        """
        Prepares the command line call to execute to start BeamNG.*.
        according to this class' and the global configuration.

        Returns:
            List of shell components ready to be called in the
            :mod:`subprocess` module.
        """
        if extensions is None:
            extensions = []

        extensions.insert(0, 'tech/techCore')
        lua = ("registerCoreModule('{}');" * len(extensions))[:-1]
        lua = lua.format(*extensions)
        call = [
            self.binary,
            '-console',
            '-rport',
            str(self.port),
            '-nosteam',
        ]

        for arg in args:
            call.append(arg)

        call_opts = {'physicsfps': '4000',
                     'lua': lua}
        if 'lua' in usr_opts.keys():
            call_opts['lua'] = usr_opts['lua']

        if 'physicsfps' in usr_opts.keys():
            call_opts['physicsfps'] = usr_opts['physicsfps']

        for key, val in call_opts.items():
            call.extend(['-' + key, val])

        if self.user:
            call.append('-userpath')
            call.append(str(self.user))
            if ' ' in str(self.user):
                msg = 'Your configured userpath contains a space. ' \
                      'Unfortunately, this is known to cause issues in ' \
                      'launching BeamNG.tech. If you require a path with a ' \
                      'space in it, you can alternatively set it manually in' \
                      'the file "startup.ini" contained in the directory of ' \
                      'your BeamNG.tech installtion. This would not be ' \
                      'automatically updated if you change the `user` ' \
                      'parameter to `BeamNGpy`, but serves as a workaround ' \
                      'until the issue is fixed in BeamNG.tech.'
                self.logger.error(msg)

        call_str = ' '.join(call)
        self.logger.debug('Created system call for starting '
                          f'BeamNG process: `{call_str}`')
        return call

    def start_beamng(self, extensions, *args, **opts):
        """
        Spawns a BeamNG.* process and retains a reference to it for later
        termination.
        """
        call = self.prepare_call(extensions, *args, **opts)
        self.process = subprocess.Popen(call)
        self.logger.info("Started BeamNG.")

    def kill_beamng(self):
        """
        Kills the running BeamNG.* process.
        """
        self.logger.info('Terminating BeamNG.tech process.')
        if self.skt:
            try:
                self.quit_beamng()
            except ConnectionResetError:
                self.skt = None

        if self.remote:
            log.warn('cannot kill remote BeamNG.research process, aborting subroutine')
            return

        if not self.process:
            return

        if os.name == "nt":
            with open(os.devnull, 'w') as devnull:
                subprocess.call([
                    'taskkill', '/F', '/T', '/PID',
                    str(self.process.pid)], stdout=devnull,
                    stderr=devnull
                )
        else:
            try:
                os.kill(self.process.pid, signal.SIGTERM)
            except:
                pass

        self.process = None

    def hello(self):
        """
        First function called after connections. Exchanges the protocol version
        with the connected simulator and raises an error upon mismatch.
        """
        data = dict(type='Hello')
        data['protocolVersion'] = PROTOCOL_VERSION
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'Hello'
        if resp['protocolVersion'] != PROTOCOL_VERSION:
            msg = 'Mismatching BeamNGpy protocol versions. ' \
                  'Please ensure both BeamNG.tech and BeamNGpy are ' \
                  'using the desired versions.\n' \
                  f'BeamNGpy\'s is: {PROTOCOL_VERSION}' \
                  f'BeamNG.tech\'s is: { resp["version"] }'
            raise BNGError(msg)
        self.logger.info('Successfully connected to BeamNG.tech.')

    def message(self, req, **kwargs):
        """
        Generic message function which is parameterized with the type of
        message to send and all parameters that are to be embedded in the
        request. Responses are expected to have the same type as the request.
        If this is not the case, an error is raised.

        Args:
            req (str): The request type.

        Raises:
            BNGValueError: If the response type does not match the request type

        Returns:
            The response received from the simulator as a dictionary.
        """
        self.logger.debug(f'Sending message of type "{req}" to '
                          'BeamNG.tech\'s game engine in blocking mode.')
        kwargs['type'] = req
        self.send(kwargs)
        resp = self.recv()
        if resp['type'] != req:
            msg = 'Got Message type "{}" but expected "{}".'
            msg = msg.format(resp['type'], req)
            raise BNGValueError(msg)
        self.logger.debug(f"Got response for message of type {req}.")

        if 'result' in resp:
            return resp['result']
        return None

    def get_levels(self):
        """
        Queries the available levels in the simulator and returns them as a
        mapping of level name to :class:`.Level` instances.

        Returns:
            A dictionary of available level names to a corresponding instance
            of the :class:`.Level` class.
        """
        levels = self.message('GetLevels')
        levels = [Level.from_dict(l) for l in levels]
        levels = {l.name: l for l in levels}
        return levels

    def get_scenarios(self, levels=None):
        """
        Queries the available scenarios and returns them as a mapping of
        paths to :class:`.Scenario` instances. The scenarios are constructed
        to point to their parent levels, so to avoid extra queries to the
        simulator about existing levels, a cache of available levels can be
        passed to this method.

        Args:
            levels (dict): A dictionary of level names to :class:`.Level`
                           instances to fill in the parent level of returned
                           scenarios.

        Returns:
            A mapping of scenario paths to their corresponding
            :class:`.Scenario` instance.
        """
        if levels is None:
            levels = self.get_levels()

        scenarios = self.message('GetScenarios')
        scenarios = [Scenario.from_dict(s) for s in scenarios]
        scenarios = {s.path: s for s in scenarios if s.level in levels.keys()}
        for _, scenario in scenarios.items():
            scenario.level = levels[scenario.level]

        return scenarios

    def get_level_scenarios(self, level):
        """
        Queries the simulator for all scenarios available in the  given level.

        Args:
            level: The level to get scenarios for. Can either be the name of
                   the level as a string or an instance of :class:`.Level`

        Returns:
            A mapping of scenario paths to their corresponding
            :class:`.Scenario` instance.
        """
        level_name = None
        if isinstance(level, Level):
            level_name = level.name
        else:
            level_name = level

        scenarios = self.message('GetScenarios')
        scenarios = [Scenario.from_dict(s) for s in scenarios]
        scenarios = {s.path: s for s in scenarios if s.level == level_name}

        for scenario in scenarios.values():
            scenario.level = level

        return scenarios

    def get_levels_and_scenarios(self):
        """
        Utility method that retrieves all levels and scenarios and returns
        them as a tuple of (levels, scenarios).

        Returns:
            (:meth:`~BeamNGpy.get_levels`, :meth:`~BeamNGpy.get_scenarios`)
        """
        levels = self.get_levels()
        scenarios = self.get_scenarios(levels=levels)

        for scenario in scenarios.values():
            level_scenarios = scenario.level.scenarios
            level_scenarios[scenario.path] = scenario

        return levels, scenarios

    def get_current_scenario(self, levels=None):
        """
        Queries the currently loaded scenario from the simulator.

        Args:
            levels (dict): A mapping of level names to :class:`.Level` instances

        Returns:
            A :class:`.Scenario` instance of the currently-loaded scenario. If
            the ``levels`` parameter contains the level the scenario is taking
            place in, the scenario's parent level field will be filled in
            accordingly.
        """
        scenario = self.message('GetCurrentScenario')
        scenario = Scenario.from_dict(scenario)

        if levels is not None:
            if scenario.level in levels:
                scenario.level = levels[scenario.level]

        return scenario

    def get_current_vehicles_info(self):
        """
        Queries the currently active vehicles in the simulator.

        Returns:
            A mapping of vehicle IDs to instances of the :class:`.Vehicle`
            class for each active vehicle. These vehicles are not connected to
            by this function.
        """
        vehicles = self.message('GetCurrentVehicles')
        return vehicles

    def get_current_vehicles(self):
        vehicles = self.get_current_vehicles_info()
        vehicles = {n: Vehicle.from_dict(v) for n, v in vehicles.items()}
        return vehicles

    def connect(self, tries=25):
        """
        Tries connecting to the running simulator over the host and port
        configuration set in this class. Upon failure, connections are
        re-attempted a limited amount of times.

        Args:
            tries (int): The amount of attempts to connect before giving up.
        """
        self.skt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.logger.info('Connecting to BeamNG.tech at: '
                         f'({self.host}, {self.port})')
        self.skt.settimeout(600)
        while tries > 0:
            try:
                self.skt.connect((self.host, self.port))
                break
            except (ConnectionRefusedError, ConnectionAbortedError) as err:
                msg = 'Error connecting to BeamNG.tech. {} tries left.'
                msg = msg.format(tries)
                self.logger.error(msg)
                self.logger.exception(err)
                sleep(5)
                tries -= 1

        self.hello()

        self.logger.info('BeamNGpy successfully connected to BeamNG.')

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

    def open(self, extensions=None, *args, launch=True, deploy=True, **opts):
        """
        Starts a BeamNG.* process, opens a server socket, and waits for the
        spawned BeamNG.* process to connect. This method blocks until the
        process started and is ready.

        Args:
            launch (bool): Whether to launch a new process or connect to a
                           running one on the configured host/port. Defaults to
                           True.
            deploy (bool): Whether to deploy the required Lua extensions as a
                           mod zip to the configured userpath. If false, it is
                           assumed that the Lua extensions are already
                           installed. Deprecated.
        """
        self.logger.info('Opening BeamNGpy instance.')

        if deploy or launch:
            if not self.effective_user:
                self.determine_effective_userpath()

        if launch:
            self.start_beamng(extensions, *args, **opts)
            sleep(10)

        self.connect()

        return self

    def close(self):
        """
        Kills the BeamNG.* process.
        """
        self.logger.info('Closing BeamNGpy instance.')
        if self.scenario:
            self.scenario.close()
            self.scenario = None

        self.kill_beamng()

    def disconnect(self):
        """
        Closes socket communication with the corresponding BeamNG instance.
        """
        if self.skt is not None:
            self.skt.close()

        self.port = None
        self.host = None
        self.skt = None

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

    def start_vehicle_connection(self, vehicle):
        """
        Prompts the simulator to initiate a new connection for the given
        vehicle, opening a server socket BeamNGpy can connect to. The port
        of this new server socket is returned.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle instance to be connected.

        Returns:
            The port connections to the given vehicle are available on.
        """
        connection_msg = {'type': 'StartVehicleConnection'}
        connection_msg['vid'] = vehicle.vid
        if vehicle.extensions is not None:
            connection_msg['exts'] = vehicle.extensions

        self.send(connection_msg)
        resp = self.recv()
        assert resp['type'] == 'StartVehicleConnection'
        vid = resp['vid']
        assert vid == vehicle.vid
        port = resp['result']
        self.logger.debug(f"Created new vehicle connection on port {port}")

        return port

    def load_scenario(self, scenario):
        """
        Loads the given scenario in the simulation and returns once loading
        is finished.

        Args:
            scenario (:class:`.Scenario`): The scenario to load.
        """
        data = {'type': 'LoadScenario', 'path': scenario.path}
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'MapLoaded'
        self.logger.info("Loaded map.")
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

        """
        flags = dict(type='EngineFlags', flags=flags)
        self.logger.debug(f'set following engine flags: {flags}')
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
        self.logger.info(f'Opened shared memory with id <{name}>, '
                         f'and of size <{size}>')
        self.send(data)

    @ack('ClosedShmem')
    def close_shmem(self, name):
        """
        Tells the simulator to close a previously-opened shared memory handle.

        Args:
            name (str): The name of the shared memory space to close.
        """
        data = dict(type='CloseShmem', name=name)
        self.logger.info(f'Closed shared memory with id <{name}>')
        self.send(data)

    @ack('OpenedAutoCamera')
    def open_auto_camera(self, name, vehicle, requested_update_time, update_priority, size, field_of_view, near_far_planes, pos, dir, is_using_shared_memory, 
        colour_shmem_handle, colour_shmem_size, annotation_shmem_handle, annotation_shmem_size, depth_shmem_handle, depth_shmem_size, is_render_colours, 
        is_render_annotations, is_render_instance, is_render_depth, is_static, is_snapping_desired, is_force_inside_triangle):

        """
        Opens a camera sensor instance in the simulator with the given parameters, either writing its data to the given shared memory space (if requested),
        or sending it through the socket directly. For argument details, see the sensor class.
        """

        data = dict(type='OpenAutoCamera')
        data['name'] = name
        data['vid'] = vehicle.vid
        data['updateTime'] = requested_update_time
        data['priority'] = update_priority
        data['size'] = size
        data['fov'] = field_of_view
        data['nearFarPlanes'] = near_far_planes
        data['pos'] = pos
        data['dir'] = dir
        data['useSharedMemory'] = is_using_shared_memory
        data['colourShmemName'] = colour_shmem_handle
        data['colourShmemSize'] = colour_shmem_size
        data['annotationShmemName'] = annotation_shmem_handle
        data['annotationShmemSize'] = annotation_shmem_size
        data['depthShmemName'] = depth_shmem_handle
        data['depthShmemSize'] = depth_shmem_size
        data['renderColours'] = is_render_colours
        data['renderAnnotations'] = is_render_annotations
        data['renderInstance'] = is_render_instance
        data['renderDepth'] = is_render_depth
        data['isStatic'] = is_static
        data['isSnappingDesired'] = is_snapping_desired
        data['isForceInsideTriangle'] = is_force_inside_triangle
        self.send(data)
        self.logger.info(f'Opened Auto Camera: "{name}')

    @ack('ClosedAutoCamera')
    def close_auto_camera(self, name):

        """
        Closes the Auto Camera instance of the given name in the simulator.

        Args:
            name (str): The name of the Auto Camera instance to close.
        """

        data = dict(type='CloseAutoCamera')
        data['name'] = name
        self.send(data)
        self.logger.info(f'Closed Auto Camera: "{name}"')

    @ack('PolledAutoCamera')
    def poll_auto_camera(self, name, is_using_shared_memory):

        """
        Polls the Auto Camera for the latest readings.

        Args:
            name (str): The name of the Camera sensor.
            is_using_shared_memory (bool): A flag which indicates if this sensor is using shared memory, or not.
        """

        # Populate a dictionary with the data needed for a request from this sensor.
        data = dict(type='PollAutoCamera')
        data['name'] = name
        data['isUsingSharedMemory'] = is_using_shared_memory

        # Send the request for updated readings to the simulation.
        self.send(data)

        # Receive the updated readings from the simulation.
        return self.recv()

    @ack('OpenedLidar')
    def open_lidar(self, name, vehicle, is_using_shared_memory, shmem_handle, shmem_size, requested_update_time, update_priority, pos, dir, 
        vertical_resolution, vertical_angle, rays_per_second, frequency, horizontal_angle, max_distance, is_visualised, is_annotated, is_static, 
        is_snapping_desired, is_force_inside_triangle):

        """
        Opens a LiDAR sensor instance in the simulator with the given parameters, either writing its data to the given shared memory space (if requested),
        or sending it directly over the socket. For argument details, see the sensor class.
        """

        data = dict(type='OpenLidar')
        data['useSharedMemory'] = is_using_shared_memory
        data['name'] = name
        data['shmem'] = shmem_handle
        data['size'] = shmem_size
        data['vid'] = vehicle.vid
        data['updateTime'] = requested_update_time
        data['priority'] = update_priority
        data['pos'] = pos
        data['dir'] = dir
        data['vRes'] = vertical_resolution
        data['vAngle'] = vertical_angle
        data['rps'] = rays_per_second
        data['hz'] = frequency
        data['hAngle'] = horizontal_angle
        data['maxDist'] = max_distance
        data['isVisualised'] = is_visualised
        data['isAnnotated'] = is_annotated
        data['isStatic'] = is_static
        data['isSnappingDesired'] = is_snapping_desired
        data['isForceInsideTriangle'] = is_force_inside_triangle
        self.send(data)
        self.logger.info(f'Opened lidar: "{name}')

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
        self.logger.info(f'Closed lidar: "{name}"')

    @ack('PolledLidar')
    def poll_lidar(self, name, is_using_shared_memory):

        """
        Polls the LiDAR sensor for the latest readings.

        Args:
            name (str): The name of the LiDAR sensor.
            is_using_shared_memory (bool): A flag which indicates if this sensor is using shared memory, or not.
        """

        # Populate a dictionary with the data needed for a request from this sensor.
        data = dict(type='PollLidar')
        data['name'] = name
        data['isUsingSharedMemory'] = is_using_shared_memory

        # Send the request for updated readings to the simulation.
        self.send(data)

        # Receive the updated readings from the simulation.
        return self.recv()

    @ack('OpenedUltrasonic')
    def open_ultrasonic(self, name, vehicle, requested_update_time, update_priority, pos, dir, size, field_of_view, near_far_planes, range_roundness, 
        range_cutoff_sensitivity, range_shape, range_focus, range_min_cutoff, range_direct_max_cutoff, sensitivity, fixed_window_size, is_visualised,
        is_static, is_snapping_desired, is_force_inside_triangle):

        """
        Opens an ultrasonic sensor in the simulator with the given parameters. For argument details, see the sensor class.
        """

        data = dict(type='OpenUltrasonic')
        data['name'] = name
        data['vid'] = vehicle.vid
        data['updateTime'] = requested_update_time
        data['priority'] = update_priority
        data['pos'] = pos
        data['dir'] = dir
        data['size'] = size
        data['fov'] = field_of_view
        data['near_far_planes'] = near_far_planes
        data['range_roundness'] = range_roundness
        data['range_cutoff_sensitivity'] = range_cutoff_sensitivity
        data['range_shape'] = range_shape
        data['range_focus'] = range_focus
        data['range_min_cutoff'] = range_min_cutoff
        data['range_direct_max_cutoff'] = range_direct_max_cutoff
        data['sensitivity'] = sensitivity
        data['fixed_window_size'] = fixed_window_size
        data['isVisualised'] = is_visualised
        data['isStatic'] = is_static
        data['isSnappingDesired'] = is_snapping_desired
        data['isForceInsideTriangle'] = is_force_inside_triangle

        self.send(data)
        self.logger.info(f'Opened ultrasonic sensor: "{name}')

    @ack('ClosedUltrasonic')
    def close_ultrasonic(self, name):

        """
        Closes the ultrasonic sensor instance of the given name in the simulator.

        Args:
            name (str): The name of the ultrasonic sensor instance to close.
        """

        data = dict(type='CloseUltrasonic')
        data['name'] = name
        self.send(data)
        self.logger.info(f'Closed ultrasonic sensor: "{name}"')

    @ack('PolledUltrasonic')
    def poll_ultrasonic(self, name):

        """
        Polls the ultrasonic sensor for the latest readings.

        Args:
            name (str): The name of the ultrasonic sensor.
        """

        # Populate a dictionary with the data needed for a request from this sensor.
        data = dict(type='PollUltrasonic')
        data['name'] = name

        # Send the request for updated readings to the simulation.
        self.send(data)

        # Receive the updated readings from the simulation.
        return self.recv()

    def teleport_vehicle(self, vehicle_id, pos, rot=None, rot_quat=None, reset=True):
        """
        Teleports the given vehicle to the given position with the given
        rotation.

        Args:
            vehicle_id (string): The id/name of the vehicle to teleport.
            pos (tuple): The target position as an (x,y,z) tuple containing
                         world-space coordinates.
            rot (tuple): Optional tuple specifying rotations around the (x,y,z)
                         axes in degrees. Deprecated.
            rot_quat (tuple): Optional tuple (x, y, z, w) specifying vehicle
                              rotation as quaternion
            reset (bool): Specifies if the vehicle will be reset to its initial
                          state during teleport (including its velocity). 

        Notes:
            The ``reset=False`` option is incompatible with setting rotation of
            the vehicle. With the current implementation, it is not possible to
            set the rotation of the vehicle and to keep its velocity during teleport.
        """
        self.logger.info(f'Teleporting vehicle <{vehicle_id}>.')
        data = dict(type='Teleport')
        data['vehicle'] = vehicle_id
        data['pos'] = pos
        data['reset'] = reset
        if rot_quat:
            data['rot'] = rot_quat
        elif rot:
            create_warning('the usage of `rot` in `beamng.teleport_vehicle` '
                           'is deprecated, the argument will be removed '
                           'in future versions',
                           DeprecationWarning)
            data['rot'] = angle_to_quat(rot)
        if not reset and (rot_quat or rot):
            create_warning('the usage of `reset=False` is incompatible with '
                           'the usage of `rot` in `beamng.teleport_vehicle`; '
                           'rotation will not be applied to the vehicle',
                           RuntimeWarning)
        self.send(data)
        response = self.recv()
        assert response['type'] == 'Teleported'
        return response['success']

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
                         axes in degrees. Deprecated.
            rot_quat (tuple): Optional tuple specifying object rotation as a
                              quaternion
        """
        data = dict(type='TeleportScenarioObject')
        data['id'] = scenario_object.id
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        elif rot:
            create_warning('the usage of `rot` in '
                           '`beamng.teleport_scenario_object` is '
                           'deprecated, the argument will be removed '
                           'in future versions',
                           DeprecationWarning)
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
        self.logger.info("Starting scenario.")

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
        self.logger.info("Restarting scenario.")

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
        self.logger.info("Stopping scenario.")

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
        not, this method resumes immediately. This can be used to queue
        commands that should be executed right after the steps have been
        simulated.

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
        self.logger.info(f"Advancing the simulation by {count} steps.")

    @ack('Paused')
    def pause(self):
        """
        Sends a pause request to BeamNG.*, blocking until the simulation is
        paused.
        """
        data = dict(type='Pause')
        self.send(data)
        self.logger.info('Pausing the simulation.')

    @ack('Resumed')
    def resume(self):
        """
        Sends a resume request to BeamNG.*, blocking until the simulation
        is resumed.
        """
        data = dict(type='Resume')
        self.send(data)
        self.logger.info('Resuming the simulation.')

    def poll_sensors(self, vehicle):
        """
        This member function is deprecated and will be removed in future
        versions. Use 'Vehicle.poll_sensors' instead.

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
        create_warning('`BeamNGpy.poll_sensors` is deprecated '
                       'and may be removed in future verions. '
                       'Use "Vehicle.poll_sensors" instead.',
                       DeprecationWarning)

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
        defining weather presets can be found in BeamNG.tech's
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
        for vehicle in self.scenario.vehicles:
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

    def get_scenetree(self):
        """
        Queries the scene tree of the current scenario/level. The scene tree
        is returned as a tree of :class:`.SceneObject` instances the root of
        which is of the ``SimGroup`` class.

        Returns:
            A tree of :class:`.SceneObject` objects that contains all objects
            in the current scene. The respective objects do not contain
            type-specific information. This information can be obtained on a
            per-object basis using :meth:`~BeamNGpy.get_scene_object_data`.
        """
        return self.message('GetSceneTree')

    def get_scene_object_data(self, obj_id):
        """
        Retrieves all available key/value pairs the simulation offers for the
        given object as a dictionary.

        Returns:
            A dictionary of key/values the simulator offers for the object of
            the given ID.
        """
        return self.message('GetObject', id=obj_id)

    def spawn_vehicle(self, vehicle, pos, rot,
                      rot_quat=(0, 0, 0, 1), cling=True):
        """
        Spawns the given :class:`.Vehicle` instance in the simulator. This
        method is meant for spawning vehicles *during the simulation*. Vehicles
        that are known to be required before running the simulation should be
        added during scenario creation instead. Cannot spawn two vehicles with
        the same id/name.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to be spawned.
            pos (tuple): Where to spawn the vehicle as a (x, y, z) triplet.
            rot (tuple): The rotation of the vehicle as a triplet of Euler
                         angles. Deprecated.
            rot_quat (tuple): Vehicle rotation in form of a quaternion
            cling (bool): If set, the z-coordinate of the vehicle's position
                          will be set to the ground level at the given
                          position to avoid spawning the vehicle below ground
                          or in the air.

        Returns:
            bool indicating whether the spawn was successful or not

        """
        data = dict(type='SpawnVehicle', cling=cling)
        data['name'] = vehicle.vid
        data['model'] = vehicle.options['model']
        data['pos'] = pos
        if rot:
            create_warning('the usage of `rot` in `beamng.spawn_vehicle` is '
                           'deprecated, the argument will be removed '
                           'in future versions',
                           DeprecationWarning)
            rot_quat = angle_to_quat(rot)
        data['rot'] = rot_quat
        data.update(vehicle.options)
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'VehicleSpawned'
        if resp['success']:
            vehicle.connect(self)
            return True
        else:
            return False

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
                         the (X, Y, Z) axes. Deprecated.
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
            create_warning('the usage of `rot` in `beamng.create_cylinder` is '
                           'deprecated, the argument will be removed '
                           'in future versions',
                           DeprecationWarning)
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
                         the (X, Y, Z) axes. Deprecated.
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
            create_warning('the usage of `rot` in `beamng.create_bump` is '
                           'deprecated, the argument will be removed '
                           'in future versions',
                           DeprecationWarning)
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
                         the (X, Y, Z) axes. Deprecated.
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
            create_warning('the usage of `rot` in `beamng.create_cone` is '
                           'deprecated, the argument will be removed '
                           'in future versions',
                           DeprecationWarning)
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
                         the (X, Y, Z) axes. Deprecated.
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
            create_warning('the usage of `rot` in `beamng.create_cube` is '
                           'deprecated, the argument will be removed '
                           'in future versions',
                           DeprecationWarning)
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
                         the (X, Y, Z) axes. Deprecated.
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
            create_warning('the usage of `rot` in `beamng.create_ring` is '
                           'deprecated, the argument will be removed '
                           'in future versions',
                           DeprecationWarning)
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
        The function is deprecated, use 'add_debug_polyline' instead!
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
                                 intensity from 0 to 1. Only the first entry is
                                 used.
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
        create_warning('`add_debug_line` is deprecated and will be '
                       'removed in future versions. '
                       'Use "add_debug_polyline" and '
                       '"add_debug_spheres" instead.',
                       DeprecationWarning)

        if spheres:
            coordinates = [s[:3] for s in spheres]
            radii = [s[3] for s in spheres]
            self.add_debug_spheres(
                coordinates, radii, sphere_colors, cling, offset)

        lineID = self.add_debug_polyline(
            points, point_colors[0], cling, offset)
        return lineID

    def remove_debug_line(self, line_id):
        create_warning('Use of `Beamngpy.remove_debug_line` is deprecated. '
                       'It will be removed in future versions. '
                       'Use `add_debug_polyline` instead.',
                       DeprecationWarning)

        self.remove_debug_polyline(line_id)

    def add_debug_spheres(self, coordinates, radii, rgba_colors,
                          cling=False, offset=0):
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

    def add_debug_polyline(self, coordinates, rgba_color,
                           cling=False, offset=0):
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

    def add_debug_text(self, origin, content, rgba_color,
                       cling=False, offset=0):
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

    def get_annotations(self):
        """
        Method to obtain the annotation configuration of the simulator.

        Returns:
            A mapping of object classes to lists containing the [R, G, B]
            values of the colors objects of that class are rendered with.
        """
        data = dict(type='GetAnnotations')
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'Annotations'
        return resp['annotations']

    def get_annotation_classes(self, annotations):
        """
        Method to convert the annotation configuration of the simulator into
        a mapping of colors to the corresponding object classes.

        Args:
            annotations (dict): The annotation configuration of the simulator.
                                Expected to be in the format
                                `get_annotations()` returns.

        Returns:
            A mapping of colors encoded as 24bit integers to object classes
            according to the simulator.
        """
        classes = {}
        for k, v in annotations.items():
            key = v[0] * 256 * 256 + v[1] * 256 + v[2]
            classes[key] = k
        return classes

    def create_scenario(self, level, name, prefab, info):
        """
        Prompts the simulator to create files required for a scenario. Namely,
        the "info.json" containing scenario information and the contents of the
        scenario prefab containing objects in the scene.

        Args:
            level (str): The name of the level the new scenario is in
            name (str): The name of the scenario
            prefab (str): Contents of the scenario's prefab file
            info (dict): Contents of the scenario's info.json
        """
        resp = self.message('CreateScenario',
                            level=level, name=name, prefab=prefab, info=info)
        return resp

    def delete_scenario(self, path):
        """
        Prompts the simulator to delete files of the scenario at the given
        path.

        Args:
            path (str): The path to the scenario relative to the
            user directory.
        """
        self.message('DeleteScenario', path=path)

    @ack('Quit')
    def quit_beamng(self):
        data = dict(type='Quit')
        self.send(data)

    def get_part_config(self, vehicle):
        """
        Retrieves the current part configuration of the given vehicle. The
        configuration contains both the current values of adjustable vehicle
        parameters and a mapping of part types to their currently-selected
        part.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to get part config of

        Returns:
            The current vehicle configuration as a dictionary.
        """
        data = dict(type='GetPartConfig')
        data['vid'] = vehicle.vid
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'PartConfig'
        resp = resp['config']
        if 'parts' not in resp or not resp['parts']:
            resp['parts'] = dict()
        if 'vars' not in resp or not resp['vars']:
            resp['vars'] = dict()
        return resp

    def get_part_options(self, vehicle):
        """
        Retrieves a mapping of part slots for the given vehicle and their
        possible parts.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to get part options of

        Returns:
            A mapping of part configuration options for the given.
        """
        data = dict(type='GetPartOptions')
        data['vid'] = vehicle.vid
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'PartOptions'
        return resp['options']

    def set_part_config(self, vehicle, cfg):
        """
        Sets the current part configuration of the given vehicle. The
        configuration is given as a dictionary containing both adjustable
        vehicle parameters and a mapping of part types to their selected parts.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to change the config of
            cfg (dict): The new vehicle configuration as a dictionary.

        Notes:
            Changing parts causes the vehicle to respawn, which repairs it as
            a side-effect.
        """
        data = dict(type='SetPartConfig')
        data['vid'] = vehicle.vid
        data['config'] = cfg
        self.send(data)
        self.await_vehicle_spawn(vehicle.vid)
        vehicle.close()
        vehicle.connect(self)

    @ack('PlayerCameraModeSet')
    def set_player_camera_mode(self, vid, mode, config):
        """
        Sets the camera mode of the vehicle identified by the given vehicle ID.
        The mode is given as a string that identifies one of the valid modes
        offered by the simulator. These modes can be queried using the
        (:meth:`~BeamNGpy.get_player_camera_mode`) method.

        The camera can be further configured with some common parameters,
        but it is not guaranteed the camera mode will respect all of them.
        These parameters include:

         * rotation: The rotation of the camera as a triplet of Euler angles
         * fov: The field of view angle
         * offset: The (x, y, z) vector to offset the camera's position by
         * distance: The distance of the camera to the vehicle

        Since each camera mode is implemented as a custom Lua extension, it is
        not possible to automatically query the exact features of the mode.
        Further information can be found in the
        lua/ge/extensions/core/cameraModes files which contain the
        implementations of each camera mode.

        Args:
            vid (str): Vehicle ID of the vehice to change the mode of.
            mode (str): Camera mode to set
            config (dict): Dictionary of further properties to set in the mode
        """
        data = dict(type='SetPlayerCameraMode')
        data['vid'] = vid
        data['mode'] = mode
        data['config'] = config
        self.send(data)

    def get_player_camera_modes(self, vid):
        """
        Retrieves information about the camera modes configured for the vehicle
        identified by the given ID.

        Args:
            vid (str): Vehicle ID of the vehicle to get camera mode information
                       of.

        Returns:
            A dictionary mapping camera mode names to configuration options.
        """
        data = dict(type='GetPlayerCameraMode')
        data['vid'] = vid
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'PlayerCameraMode'
        return resp['cameraData']

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
