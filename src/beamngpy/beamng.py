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

from pathlib import Path
from threading import Thread

import msgpack

from PIL import Image

from .beamngenv import CFG as cfg
from .beamngenv import ENV as env
from .communication import send_msg, recv_msg

VERSION = 'v1.0'

BUF_SIZE = 65536
IMG_CHANNELS = 'RGBA'
MAX_QUEUE = 1024
BNG_HOME = env['BNG_HOME']

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


class BNGError(Exception):
    """
    Generic BeamNG error
    """
    pass


class BNGValueError(ValueError):
    """
    Value error specific to BeamNGpy.
    """
    pass


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

    def decode_msg(self, msg):
        """
        Performs appropriate decoding of the given dict depending on the type
        specified within. For example, ``VehicleState`` messages will have
        their raw image data decoded to a :py:class:`PIL.Image` instance.
        Decoding is done in place.

        Args:
            msg (dict): Dictionary to decode. Must contain at least a ``type``
                        entry.

        Returns:
            Decoded message as a :py:class:`dict`.
        """
        if msg["type"] == "VehicleState":
            img_w = msg["view"]["width"]
            img_h = msg["view"]["height"]
            img_d = base64.b64decode(msg["view"]["pixelsBase64"])
            img_d = np.frombuffer(img_d, dtype=np.uint8)
            img_d = img_d.reshape(img_h, img_w, len(IMG_CHANNELS))

            msg["img"] = Image.fromarray(img_d)

        return msg

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
        self.client, addr = self.server.accept()

        log.debug('Connection established. Awaiting "hello"...')
        hello = self.recv()
        assert hello['type'] == 'Hello'
        if hello['version'] != VERSION:
            print('BeamNGpy and BeamNG.* version mismatch: '
                  'BeamNGpy {}, BeamNG.* {}'.format(hello['version'], VERSION))
            print('Make sure both this library and BeamNG.* are up to date.')
            print('Operation will proceed, but some features might not work.')

        log.info('Started BeamNPy communicating on %s', addr)

    def close(self):
        """
        Kills the BeamNG.drive process and closes the server.
        """
        log.info('Closing BeamNGpy instance...')
        self.server.close()
        self.kill_beamng()

    def vcontrol(self, inputs):
        """
        Sends a vehicle control package to the client. The inputs are to be
        specified as a dictionary specifying values to be set for each type of
        input. Possible inputs are:

            * ``steering``: Steering angle, within [-1.0, 1.0], negative = left
            * ``throttle``: Throttle value, within [0.0, 1.0]
            * ``brake``: Brake intensity, within [0.0, 1.0]
            * ``parkingbrake``: Parking brake intensity, within [0.0, 1.0]
            * ``gear``: The gear to shift to. The vehicle needs to be shifted
                        into at least 1 to accelerate forward.

        This command will only change inputs contained in the dictionary. If no
        change to, for example, the throttle value is supposed to happen, don't
        specify any key/value pair for it.

        Args:
            inputs (dict): Dictionary of inputs to set, as described above.

        Examples:
            Go full throttle whilst steering all the way to the left:

            .. code-block:: python

                bpy.vcontrol({'throttle': 1., 'steering': -1.})

            Now brake slightly:

            .. code-block:: python

                bpy.vcontrol({'brake': 0.2})

            Stop everything:

            .. code-block:: python

                bpy.vcontrol({'throttle': 0, 'steering': 0, 'brake': 0})

        """
        inputs = {"type": "VControl", "inputs": inputs}
        self.send(inputs)

    def req_vstate(self, width=None, height=None):
        """
        Sends a request for the ego vehicle's current state that, once the
        client responds, will be retrievable via :py:meth:`BeamNGPy.poll`.

        The state will be a dictionary containing the following entries:

            * ``pos``: List of [x, y, z] coordinates of the vehicle.
            * ``vel``: List of [x, y, z] velocities of the vehicle in metres
                       per second.
            * ``dir``: List of [x, y, z] components of the direction vector of
                       the vehicle; the vector is normalised.
            * ``rot``: Rotation of the vehicle in degrees.
            * ``steering``: Current steering angle, within [-1.0, 1.0]
            * ``throttle``: Current throttle intensity, within [0.0, 1.0]
            * ``brake``: Current brake intensity, within [0.0, 1.0]
            * ``parkingbrake``: Current parkingbrake value, within [0.0, 1.0]
            * ``gear``: Current gear the vehicle is shifted to.
            * ``img``: Current screenshot of the game as a
                       :py:class:`PIL.Image` instance. The dimensions of this
                       image can be controlled with the ``width`` and
                       ``height`` arguments.

        Args:
            width (int): Width of the screenshot. If ``None``, it will be
                         calculated based on the given height such that the
                         aspect ratio of the game is maintained.
            height (int): Height of the screenshot. If ``None``, it will be
                          calculated based on the given width such taht the
                          aspect ratio of the game is maintained.

        Raises:
            BNGValueError: If both ``width`` and ``height`` are ``None`` or 0.

        """
        if not width and not height:
            raise BNGValueError("Need to specify at least height or width.")

        data = dict()
        data["type"] = "ReqVState"
        data["width"] = width
        data["height"] = height

        self.send(data)

    def get_vstate(self, width=None, height=None):
        """
        Same as :py:meth:`BeamNGPy.req_vstate` but blocks until the vehicle
        state was received and returns it.

        Args:
            width (int): Same as ``width`` in :py:meth:`BeamNGPy.req_vstate`
            height (int): Same as ``height`` in :py:meth:`BeamNGPy.req_vstate`

        Returns:
            The vehicle state as described in :py:meth:`BeamNGPy.req_vstate`.
        """
        self.req_vstate(width=width, height=height)
        while True:
            vstate = self.poll()
            if vstate["type"] == "VehicleState":
                return vstate

    def change_vcursor(self, cursor=0):
        """
        Changes the vehicle cursor to focus on the given vehicle number. By
        default, the simulation focuses on the first vehicle with number 0,
        changing this to, for example, 1 will make methods like
        :py:meth:`BeamNGPy.get_vstate` focus on vehicle 1 instead.

        This method blocks until the change is acknowledged by the simulation.

        Args:
            cursor (int): The new vehicle cursor. Defaults to 0, which is the
                          first spawned car in BeamNG.drive.
        """
        data = dict()
        data["type"] = "VehicleCursor"
        data["cursor"] = cursor
        self.send(data)
        while True:
            state = self.poll()
            if state["type"] == "VehicleCursor":
                assert cursor == state["cursor"]
            return

    def relative_camera(self, pos=(0, 0, 0), rot=(0, 0, 0), fov=90):
        """
        Switches the camera to one relative to the vehicle. The given
        parameters control position, orientation, and field of view of the
        camera.

        Args:
            pos (tuple): X, Y, Z coordinates of the camera's position relative
                         to the vehicle
            rot (tuple): Pitch, Yaw, Roll angles of the camera's orientation
                         relative to the vehicle
            fov (float): Field of View

        Examples:
            Get a bumper cam on the ETK800 Series:

            .. code-block:: python

                pos = (-0.5, 2, 0.5)
                rot = (180, 0, 180)
                fov = 90
                bpy.relative_camera(pos, rot, fov)

        """
        data = dict()
        data["type"] = "RelativeCamera"
        data["pos"] = pos
        data["rot"] = rot
        data["fov"] = fov
        self.send(data)

    def hide_hud(self):
        """
        Hides the HUD.
        """
        data = dict()
        data["type"] = "HideHUD"
        self.send(data)

    def show_hud(self):
        """
        Shows the HUD again.
        """
        data = dict()
        data["type"] = "ShowHUD"
        self.send(data)

    def connect_vehicle(self, vehicle):
        vehicle_port = self.next_port
        self.next_port += 1
        vehicle_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        vehicle_server.bind(self.host, vehicle_port)
        vehicle_server.listen()
        log.debug('Starting vehicle server for %s on: %s:%s',
                  vehicle, self.host, vehicle_port)

        connection_msg = {'type': 'VehicleConnection'}
        connection_msg['vid'] = vehicle.vid
        connection_msg['host'] = self.host
        connection_msg['port'] = self.vehicle_port
        self.send(connection_msg)
        return vehicle_server

    def setup_vehicles(self):
        vehicles = self.scenario.vehicles
        for vehicle in vehicles.keys():
            vehicle_server = self.connect_vehicle(vehicle)
            vehicle.setup(vehicle_server)

    def load_scenario(self, scenario):
        info_path = scenario.get_info_path()
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

        data = {"type": "SetPositionRotation"}
        data["pos"] = pos
        data["rot"] = rot
        self.send(data)
        while True:
            resp = self.poll()
            if resp["type"] == "VehicleMoved":
                return

    def start_scenario(self):
        """
        Starts the scenario; equivalent to clicking the "Start" button in the
        game after loading a scenario. This method blocks until the countdown
        to the scenario's start has finished.
        """
        data = {"type": "StartScenario"}
        self.send(data)
        while True:
            resp = self.poll()
            if resp["type"] == "ScenarioStarted":
                return

    def restart_scenario(self):
        """
        Restarts a running scenario
        """
        data = {"type": "RestartScenario"}
        self.send(data)
        while True:
            resp = self.poll()
            if resp["type"] == "ScenarioRestarted":
                return

    def pause(self):
        """
        Sends a pause request to BeamNG.drive, blocking until the simulation is
        paused.
        """
        data = dict()
        data["type"] = "Pause"
        self.send(data)
        while True:
            resp = self.poll()
            if resp["type"] == "Paused":
                return

    def resume(self):
        """
        Sends a resume request to BeamNG.drive, blocking until the simulation
        is resumed.
        """
        data = dict()
        data["type"] = "Resume"
        self.send(data)
        while True:
            resp = self.poll()
            if resp["type"] == "Resumed":
                return

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
