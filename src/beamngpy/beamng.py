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

from threading import Thread

import msgpack

from PIL import Image

from .bnpcfg import CFG as cfg

BUF_SIZE = 65536
IMG_CHANNELS = 'RGBA'
MAX_QUEUE = 1024


class BeamNGPy:
    """
    This class provides methods used to start, interact with, and terminate an
    instance of BeamNG.drive. It enables launching a BeamNG.drive process that
    then stands by for further instructions, including loading of scenarios,
    controlling vehicles, and extracting data from the simulation.

    .. note::
        A BeamNG.drive process stores data locally in a so called ``userpath``.
        Multiple instances should not be set to run in the same userpath, as
        sharing those resources leads to undefined behaviour. Instead, when
        using multiple BeamNG.drive processes, specify unique user paths in the
        constructor.

    Args:
        host (str): Hostname or IP,
                    see :py:mod:`socket`, :py:data:`socket.AF_INET`
        post (int): Port to listen on,
                    see :py:mod:`socket`, :py:data:`socket.AF_INET`
        userpath (str, optional): The userpath BeamNG.drive should run in.
                                  If not specified, BeamNG.drive's default will
                                  be used.
        binary (str, optional): Name of the BeamNG.drive binary to call.


    Attributes:
        host (str): Host given during construction.
        port (int): Port given during construction.
        server (:py:class:`socket.socket`): Server socket clients connect to.
        userpath (str): User path to run BeamNG.drive in. Can be ``None`` to use
                        the default.
        binary (str): The BeamNG.drive binary that will be called. If ``None``,
                      no binary will be called and it is assumed that an
                      instance is already running.
        process (:py:class:`subprocess.Popen`): BeamNG.drive process if one has
                                                been started yet, ``None``
                                                otherwise.
        client (:py:class:`socket.socket`): Currently connected client if one
                                            has connected yet, ``None`` otherwise.
        queue (:py:class:`queue.Queue`): Messages received from the socket, but
                                         yet to be retrieved via either
                                         :py:meth:`BeamNGPy.poll` or
                                         :py:meth:`BeamNGPy.poll_all`.
        worker (:py:class:`threading.Thread`): Worker thread reading and parsing
                                               msgpack messages from the client
                                               socket.

    Examples:
        Set up a BeamNGPy instance on localhost and port 64256, using the
        default ``userpath`` and load the scenario ``levels/test/test.json``:

        .. code-block:: python

            bpy = beamngpy.BeamNGPy('localhost', 64256)
            bpy.open()
            bpy.load_scenario('levels/test/test.json')
            bpy.start_scenario()
            # Do stuff...
            bpy.close()

        Set up a BeamNGPy instance with a custom ``userpath``, load a scenario,
        drive the vehicle for 10s, retrieve its state, and save the screenshot
        to disk:

        .. code-block:: python

            with BeamNGPy('localhost', 64256, userpath='C:/bpy/') as bpy:
                bpy.load_scenario('levels/test/test.json')
                bpy.start_scenario()
                bpy.vcontrol({'throttle': 1.0, 'steering': 0.25})
                time.sleep(10)
                bpy.req_vstate()
                vstate = bpy.poll()
                vstate['img'].save('scrn.png')

    """

    def __init__(self, host, port, userpath=None, binary=cfg.beamng_binary):
        self.host = host
        self.port = port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.userpath = userpath
        self.binary = binary

        self.process = None
        self.client = None
        self.queue = queue.Queue(maxsize=MAX_QUEUE)

        self.worker = None

    def prepare_call(self):
        """
        Prepares the command line call to execute to start BeamNG.drive
        according to this class' and the global configuration.

        Returns:
            List of shell components ready to be called in the
            :py:mod:`subprocess` module.
        """
        call = [
            self.binary,
            "-lua",
            "registerCoreModule('{}')".format(cfg.beamng_extension),
        ]

        if self.userpath:
            call.append('-userpath')
            call.append(self.userpath)

        return call

    def start_beamng(self):
        """
        Spawns a BeamNG.drive process and retains a reference to it for later
        termination.
        """
        call = self.prepare_call()
        log.debug("Starting BeamNG process...")
        self.process = subprocess.Popen(call)

    def kill_beamng(self):
        """
        Kills the running BeamNG.drive process.
        """
        log.debug('Killing BeamNG process...')
        if os.name == "nt":
            subprocess.call(
                ["taskkill", "/F", "/T", "/PID", str(self.process.pid)])
        else:
            os.kill(self.process.pid, signal.SIGTERM)
        self.process = None

    def start_server(self):
        """
        Binds a server socket to the configured host & port and starts listening
        on it.
        """
        self.server.bind((self.host, self.port))
        self.server.listen()
        log.info("Started BeamNPy server on %s:%s", self.host, self.port)

    def open(self):
        """
        Starts a BeamNG.drive process, opens a server socket, and waits for the
        spawned BeamNG.drive process to connect. This method blocks until the
        process started and is ready.
        """
        log.info("Opening BeamNPy instance...")
        self.start_server()
        if self.binary:
            self.start_beamng()
        self.client, addr = self.server.accept()
        self.sfile = self.client.makefile(mode='rwb')

        self.worker = Thread(target=self.recv, daemon=True)
        self.worker.start()
        log.info("Started BeamNPy communicating on %s", addr)

    def close(self):
        """
        Kills the BeamNG.drive process and closes the server.
        """
        log.info("Closing BeamNPy instance...")
        self.server.close()
        self.kill_beamng()

    def send(self, data):
        """
        Sends the given dictionary to the client, automatically encoding it with
        :py:mod`msgpack` before transmission.

        Args:
            data (dict): Data to send.
        """
        data = msgpack.packb(data, use_bin_type=True)
        self.client.send(data)
        self.client.send(bytes("\n", "ascii"))  # Lua socket reads line-by-line

    def vcontrol(self, inputs):
        """
        Sends a vehicle control package to the client. The inputs are to be
        specified as a dictionary specifying values to be set for each type of
        input. Possible inputs are:

            * ``steering``: Steering angle, within [-1.0, 1.0], negative goes left
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
            * ``vel``: List of [x, y, z] velocities of the vehicle in metres per
                     second.
            * ``dir``: List of [x, y, z] components of the direction vector of the
                     vehicle; the vector is normalised.
            * ``rot``: Rotation of the vehicle in degrees.
            * ``steering``: Current steering angle, within [-1.0, 1.0]
            * ``throttle``: Current throttle intensity, within [0.0, 1.0]
            * ``brake``: Current brake intensity, within [0.0, 1.0]
            * ``parkingbrake``: Current parkingbrake intensity, within [0.0, 1.0]
            * ``gear``: Current gear the vehicle is shifted to.
            * ``img``: Current screenshot of the game as a :py:class:`PIL.Image`
                       instance. The dimensions of this image can be controlled
                       with the ``width`` and ``height`` arguments.

        Args:
            width (int): Width of the screenshot. If ``None``, it will be
                         calculated based on the given height such that the
                         aspect ratio of the game is maintained.
            height (int): Height of the screenshot. If ``None``, it will be
                          calculated based on the given width such taht the
                          aspect ratio of the game is maintained.

        Raises:
            ValueError: If both ``width`` and ``height`` are ``None`` or 0.

        """
        if not width and not height:
            raise ValueError("Need to specify at least height or width.")

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
            width (int): Same as the ``width`` in :py:meth:`BeamNGPy.req_vstate`
            height (int): Same as the ``height`` in
                          :py:meth:`BeamNGPy.req_vstate`

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
        Switches the camera to one relative to the vehicle. The given parameters
        control position, orientation, and field of view of the camera.

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

    def load_scenario(self, path):
        """
        Loads a scenario identified by the given path. This method blocks until
        BeamNG.drive has finished loading the map & scenario and is ready to
        start.

        Args:
            path (str): Path to the scenario descriptor to load. Must point to a
                        the file within BeamNG.drive's userpath.

        Examples:
            Load an example scenario.

            .. code-block:: python

                bpy.load_scenario('levels/example/scenarios/example.json')

        """
        data = {"type": "LoadScenario", "path": path}
        self.send(data)
        while True:
            resp = self.poll()
            if resp["type"] == "MapLoaded":
                return

    def start_scenario(self):
        """
        Starts the scenario; equivalent to clicking the "Start" button in the
        game after loading a scenario. This method blocks until the countdown to
        the scenario's start has finished.
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
        Sends a resume request to BeamNG.drive, blocking until the simulation is
        resumed.
        """
        data = dict()
        data["type"] = "Resume"
        self.send(data)
        while True:
            resp = self.poll()
            if resp["type"] == "Resumed":
                return

    def decode_msg(self, msg):
        """
        Performs appropriate decoding of the given dict depending on the type
        specified within. For example, ``VehicleState`` messages will have their
        raw image data decoded to a :py:class:`PIL.Image` instance. Decoding is
        done in place.

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

    def recv(self):
        """
        Indefinitely reads :py:mod:`msgpack`-encoded dictionaries from the
        client socket and enqueues them onto :py:attr:`BeamNGPy.queue`. Meant to
        be called in a separate thread.
        """
        unpacker = msgpack.Unpacker(encoding='utf-8')
        while True:
            try:
                data = self.client.recv(BUF_SIZE)
            except socket.error:
                break

            unpacker.feed(data)
            for msg in unpacker:
                log.debug("Queueing new message: %s", msg)
                self.queue.put(msg)

    def poll(self):
        """
        Polls the next message from the message queue and returns it. If nothing
        is on the queue, this method blocks until a message is received.
        Messages will be parsed as :py:class:`dict` and decoded using
        :py:meth:`BeamNGPy.decode_msg` before returning.

        Returns:
            Next message as a :py:class:`dict` with appropriate decoding already
            performed.

        Examples:
            Request the vehicle state and retrieve it once it was sent by the
            client:

            .. code-block:: python

                bpy.req_vstate()
                vstate = bpy.poll()
        """
        msg = self.queue.get()
        if 'type' in msg:
            msg = self.decode_msg(msg)
        return msg

    def poll_all(self):
        """
        Returns a generator to all messages on the queue in the order they were
        received, ending once the queue is empty. New messages arriving in the
        lifetime of this generator will be included.

        Examples:
            Get the vehicle state every second for ten seconds, then save each
            frame to disk:

            .. code-block:: python

                for _ in range(10):
                    bpy.req_vstate()
                    time.sleep(1)

                for i, vstate in enumerate(bpy.poll_all()):
                    vstate['img'].save(f'{i:02}.png')
        """
        while not self.queue.empty():
            yield self.queue.get()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
