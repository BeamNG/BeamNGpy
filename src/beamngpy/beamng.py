"""
.. module:: beamng
    :platform: Windows
    :synopsis: Contains the main :py:class:`.BeamNGPy` class used to interface
               with BeamNG.drive.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""

import base64
import logging as log
import mmap
import numpy as np
import os
import queue
import signal
import socket
import subprocess
import sys
import time

from pathlib import Path
from threading import Thread
from time import sleep

import msgpack

from PIL import Image

from .beamng_common import ack
from .beamng_common import *

VERSION = 'v1.1'

BINARIES = [
    'Bin64/BeamNG.research.x64.exe',
    'Bin64/BeamNG.drive.x64.exe',
]


def log_exception(extype, value, trace):
    """
    Hook to log uncaught exceptions to the logging framework. Register this as
    the excepthook with `sys.excepthook = log_exception`.
    """
    log.exception("Uncaught exception: ", exc_info=(extype, value, trace))


def setup_logging(log_file=None):
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

    log.info('Started BeamNGpy logging.')


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
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.home = home
        if not self.home:
            self.home = ENV['BNG_HOME']
        if not self.home:
            raise BNGValueError('No BeamNG home folder given. Either specify '
                                'one in the constructor or define an '
                                'environment variable "BNG_HOME" that '
                                'points to where your copy of BeamNG.* is.')
        self.home = Path(self.home).resolve()
        if user:
            self.user = Path(user).resolve()
        else:
            self.user = None

        self.process = None
        self.skt = None

        self.scenario = None

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

    def prepare_call(self):
        """
        Prepares the command line call to execute to start BeamNG.*.
        according to this class' and the global configuration.

        Returns:
            List of shell components ready to be called in the
            :mod:`subprocess` module.
        """
        binary = self.determine_binary()
        call = [
            binary,
            '-console',
            '-rport',
            str(self.port),
            '-rhost',
            str(self.host),
            '-lua',
            "registerCoreModule('{}')".format('util_researchGE'),
        ]

        if self.user:
            call.append('-userpath')
            call.append(str(self.user))

        return call

    def start_beamng(self):
        """
        Spawns a BeamNG.* process and retains a reference to it for later
        termination.
        """
        call = self.prepare_call()
        log.debug('Starting BeamNG process...')
        self.process = subprocess.Popen(call)

    def kill_beamng(self):
        """
        Kills the running BeamNG.* process.
        """
        if not self.process:
            return

        log.debug('Killing BeamNG process...')
        if os.name == "nt":
            subprocess.call(
                ['taskkill', '/F', '/T', '/PID', str(self.process.pid)])
        else:
            os.kill(self.process.pid, signal.SIGTERM)

        self.process = None

    def start_server(self):
        """
        Binds a server socket to the configured host & port and starts
        listening on it.
        """
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

    def open(self, launch=True):
        """
        Starts a BeamNG.* process, opens a server socket, and waits for the
        spawned BeamNG.* process to connect. This method blocks until the
        process started and is ready.
        """
        log.info('Opening BeamNPy instance...')
        self.start_server()
        if launch:
            self.start_beamng()
        self.skt, addr = self.server.accept()

        log.debug('Connection established. Awaiting "hello"...')
        hello = self.recv()
        assert hello['type'] == 'Hello'
        if hello['version'] != VERSION:
            print('BeamNGpy and BeamNG.* version mismatch: '
                  'BeamNGpy {}, BeamNG.* {}'.format(hello['version'], VERSION))
            print('Make sure both this library and BeamNG.* are up to date.')
            print('Operation will proceed, but some features might not work.')

        log.info('Started BeamNGpy communicating on %s', addr)
        return self

    def close(self):
        """
        Kills the BeamNG.* process and closes the server.
        """
        log.info('Closing BeamNGpy instance...')
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

    def connect_vehicle(self, vehicle):
        """
        Creates a server socket for the given vehicle and sends a connection
        request for it to the simulation. This method does not wait for the
        connection to be established but rather returns the respective server
        socket to the caller.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle instance to be connected.

        Returns:
            The server socket created and waiting for a conection.
        """
        vehicle_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        vehicle_server.bind((self.host, self.next_port))
        vehicle_server.listen()
        log.debug('Starting vehicle server for %s on: %s:%s',
                  vehicle, self.host, self.next_port)

        connection_msg = {'type': 'VehicleConnection'}
        connection_msg['vid'] = vehicle.vid
        connection_msg['host'] = self.host
        connection_msg['port'] = self.next_port
        self.send(connection_msg)
        self.next_port += 1
        return vehicle_server

    def setup_vehicles(self, scenario):
        """
        Goes over the current scenario's vehicles and establishes a connection
        between their vehicle instances and the vehicles in simulation. Engine
        flags required by the vehicles' sensor setups are sent and connect-
        hooks of the respective sensors called upon connection. This method
        blocks until all vehicles are fully connected.
        """
        vehicles = scenario.vehicles
        for vehicle in vehicles.keys():
            flags = vehicle.get_engine_flags()
            self.set_engine_flags(flags)
            vehicle_server = self.connect_vehicle(vehicle)
            vehicle.connect(self, vehicle_server)

    def load_scenario(self, scenario):
        """
        Loads the given scenario in the simulation and returns once loading
        is finished.

        Args:
            scenario (:class:`.Scenario`): The scenario to load.
        """
        info_path = scenario.get_info_path()
        info_path = info_path.replace(str(self.home), '')
        info_path = info_path[1:]
        info_path = info_path.replace('\\', '/')
        data = {'type': 'LoadScenario', 'path': info_path}
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'MapLoaded'
        sleep(12.0)
        self.setup_vehicles(scenario)
        self.scenario = scenario

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

    @ack('ScenarioStarted')
    def start_scenario(self):
        """
        Starts the scenario; equivalent to clicking the "Start" button in the
        game after loading a scenario. This method blocks until the countdown
        to the scenario's start has finished.
        """
        if not self.scenario:
            raise BNGError('Need to load a scenario before starting one.')

        flags = self.scenario.get_engine_flags()
        if flags:
            self.set_engine_flags(flags)
        self.scenario.connect(self)

        data = dict(type="StartScenario")
        self.send(data)

    @ack('ScenarioRestarted')
    def restart_scenario(self):
        """
        Restarts a running scenario.
        """
        data = dict(type='RestartScenario')
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

    @ack('Stepped')
    def step(self, count):
        data = dict(type='Step', count=count)
        self.send(data)

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
        engine_reqs, vehicle_reqs = vehicle.encode_sensor_requests()
        sensor_data = dict()

        if engine_reqs['sensors']:
            start = time.time()
            self.send(engine_reqs)
            response = self.recv()
            assert response['type'] == 'SensorData'
            sensor_data.update(response['data'])

        if vehicle_reqs['sensors']:
            response = vehicle.poll_sensors(vehicle_reqs)
            sensor_data.update(response)

        result = vehicle.decode_sensor_response(sensor_data)
        vehicle.sensor_cache = result
        return result

    def render_cameras(self):
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
        data = dict(type="GetDecalRoadVertices")
        self.send(data)
        response = self.recv()
        assert response['type'] == 'DecalRoadVertices'
        roads_verts = response['vertices']
        roads = dict()
        for road, road_verts in roads_verts.items():
            vertices = list()
            for i in range(0, len(road_verts), 3):
                vertex = (road_verts[i], road_verts[i + 1], road_verts[i + 2])
                vertices.append(vertex)
            edges = list()
            for i in range(0, len(vertices), 3):
                edge = (vertices[i], vertices[i + 1], vertices[i + 2])
                edges.append(edge)
            roads[road] = edges

        return roads

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
