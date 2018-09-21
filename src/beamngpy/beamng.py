"""
.. module:: beamng
    :platform: Windows
    :synopsis: Contains the main :py:class:`.BeamNGPy` class used to interface
               with BeamNG.drive.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""

import base64
import logging as log
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

VERSION = 'v1.0'

BNG_HOME = ENV['BNG_HOME']

BINARIES = [
    'Bin64/BeamNG.drive.x64.exe',
    'Bin64/BeamNG.research.x64.exe',
    'Bin32/BeamNG.drive.x86.exe',
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

    log.info('Started BeamNGPy logging.')


class BeamNGpy:

    def __init__(self, host, port, home=BNG_HOME):
        self.host = host
        self.port = port
        self.next_port = self.port + 1
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.home = home
        if not self.home:
            raise BNGValueError('No BeamNG home folder given. Either specify '
                                'one in the constructor or define an '
                                'environment variable "BNG_HOME" that '
                                'points to where your copy of BeamNG.* is.')
        self.home = Path(self.home).resolve()

        self.process = None
        self.skt = None

        self.scenario = None

    def determine_binary(self):
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
        Prepares the command line call to execute to start BeamNG.drive
        according to this class' and the global configuration.

        Returns:
            List of shell components ready to be called in the
            :py:mod:`subprocess` module.
        """
        binary = self.determine_binary()
        call = [
            binary,
            '-rport',
            str(self.port),
            '-rhost',
            str(self.host),
            '-lua',
            "registerCoreModule('{}')".format('util_researchGE'),
        ]

        return call

    def start_beamng(self):
        """
        Spawns a BeamNG.drive process and retains a reference to it for later
        termination.
        """
        call = self.prepare_call()
        log.debug('Starting BeamNG process...')
        self.process = subprocess.Popen(call)

    def kill_beamng(self):
        """
        Kills the running BeamNG.drive process.
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
        log.info('Started BeamNPy server on %s:%s', self.host, self.port)

    def send(self, data):
        return send_msg(self.skt, data)

    def recv(self):
        return recv_msg(self.skt)

    def open(self, launch=True):
        """
        Starts a BeamNG.drive process, opens a server socket, and waits for the
        spawned BeamNG.drive process to connect. This method blocks until the
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

        log.info('Started BeamNPy communicating on %s', addr)
        return self

    def close(self):
        """
        Kills the BeamNG.drive process and closes the server.
        """
        log.info('Closing BeamNGpy instance...')
        self.server.close()
        self.kill_beamng()

    def hide_hud(self):
        """
        Hides the HUD.
        """
        data = dict(type='HideHUD')
        self.send(data)

    def show_hud(self):
        """
        Shows the HUD again.
        """
        data = dict(type='ShowHUD')
        self.send(data)

    def connect_vehicle(self, vehicle):
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

    def setup_vehicles(self):
        vehicles = self.scenario.vehicles
        for vehicle in vehicles.keys():
            flags = vehicle.get_engine_flags()
            self.set_engine_flags(flags)
            vehicle_server = self.connect_vehicle(vehicle)
            vehicle.connect(self, vehicle_server)

    def load_scenario(self, scenario):
        info_path = scenario.get_info_path()
        info_path = info_path.replace(str(self.home), '')
        info_path = info_path[1:]
        info_path = info_path.replace('\\', '/')
        data = {'type': 'LoadScenario', 'path': info_path}
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'MapLoaded'
        sleep(10.0)
        self.scenario = scenario
        self.setup_vehicles()

    def move_vehicle(self, pos, rot):
        """
        Moves the current vehicle to the given position with the given
        rotation. Blocks until the client has moved the vehicle.

        Args:
            pos (tuple): A tuple of three coordinates encoding the x, y, z
                         position the vehicle is supposed to be moved to.
            rot (tuple): A tuple of three angles encoding the pitch, yaw, roll
                         rotations to apply to the vehicle.

        Raises:
            BNGValueError: If position or rotation don't have three components.
        """
        if len(pos) != 3:
            raise BNGValueError("Position must have three components.")
        if len(rot) != 3:
            raise BNGValueError("Rotation must have three components.")

        data = dict(type="SetPositionRotation")
        data["pos"] = pos
        data["rot"] = rot
        self.send(data)
        while True:
            resp = self.poll()
            if resp["type"] == "VehicleMoved":
                return

    @ack('SetEngineFlags')
    def set_engine_flags(self, flags):
        flags = dict(type='EngineFlags', flags=flags)
        self.send(flags)

    @ack('OpenedShmem')
    def open_shmem(self, name, size):
        data = dict(type='OpenShmem', name=name, size=size)
        self.send(data)

    @ack('ClosedShmem')
    def close_shmem(self, name):
        data = dict(type='CloseShmem', name=name)
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
        Restarts a running scenario
        """
        data = dict(type='RestartScenario')
        self.send(data)

    @ack('SetPhysicsDeterministic')
    def set_deterministic(self):
        data = dict(type='SetPhysicsDeterministic')
        self.send(data)

    @ack('SetPhysicsNonDeterministic')
    def set_nondeterministic(self):
        data = dict(type='SetPhysicsNonDeterministic')
        self.send(data)

    @ack('SetFPSLimit')
    def set_steps_per_second(self, sps):
        data = dict(type='FPSLimit', fps=sps)
        self.send(data)

    @ack('RemovedFPSLimit')
    def remove_step_limit(self):
        data = dict(type='RemoveFPSLimit')
        self.send(data)

    @ack('Stepped')
    def step(self, count):
        data = dict(type='Step', count=count)
        self.send(data)

    @ack('Paused')
    def pause(self):
        """
        Sends a pause request to BeamNG.drive, blocking until the simulation is
        paused.
        """
        data = dict(type='Pause')
        self.send(data)

    @ack('Resumed')
    def resume(self):
        """
        Sends a resume request to BeamNG.drive, blocking until the simulation
        is resumed.
        """
        data = dict(type='Resume')
        self.send(data)

    def poll_sensors(self, vehicle):
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

    def update(self, scenario):
        scenario.update(self)

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
