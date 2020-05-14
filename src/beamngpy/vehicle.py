"""
.. module:: vehicle
    :platform: Windows
    :synopsis: Contains vehicle-related classes/functions  for BeamNGpy.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""

import base64
import logging as log

from .beamngcommon import *


SHIFT_MODES = {
    'realistic_manual': 0,
    'realistic_manual_auto_clutch': 1,
    'arcade': 2,
    'realistic_automatic': 3,
}


class Vehicle:
    """
    The vehicle class represents a vehicle of the simulation that can be
    interacted with from BeamNGpy. This class offers methods to both control
    the vehicle's state as well as retrieve information about it through
    sensors the user can attach to the vehicle.
    """

    def __init__(self, vid, **options):
        """
        Creates a vehicle with the given vehicle ID. The ID must be unique
        within the scenario.

        Args:
            vid (str): The vehicle's ID.
        """
        self.vid = vid

        self.port = None

        self.bng = None
        self.server = None
        self.skt = None

        self.sensors = dict()

        options['color'] = options.get('colour', options.get('color', None))
        self.options = options

        self.sensor_cache = dict()

        self.state = None
        """
        This field contains the vehicle's current state in the running
        scenario. It is None if no scenario is running or the state has not
        been retrieved yet. Otherwise, it contains the following key entries:

         * ``pos``: The vehicle's position as an (x,y,z) triplet
         * ``dir``: The vehicle's direction vector as an (x,y,z) triplet
         * ``up``: The vehicle's up vector as an (x,y,z) triplet
         * ``vel``: The vehicle's velocity along each axis in metres per
                    second as an (x,y,z) triplet

        Note that the `state` variable represents a *snapshot* of the last
        state. It has to be updated through :meth:`.Vehicle.update_vehicle`,
        which is made to retrieve the current state. Alternatively, for
        convenience, a call to :meth:`.Vehicle.poll_sensors` also updates the
        vehicle state along with retrieving sensor data.
        """

    def __hash__(self):
        return hash(self.vid)

    def __eq__(self, other):
        if isinstance(other, type(self)):
            return self.vid == other.vid
        return False

    def __str__(self):
        return 'V:{}'.format(self.vid)

    def send(self, data):
        """
        Sends the given data as a message to the corresponding vehicle's
        socket.
        """
        return send_msg(self.skt, data)

    def recv(self):
        """
        Reads a message from the corresponding vehicle's socket and returns it
        as a dictionary.

        Returns:
            The message received as a dictionary.
        """
        return recv_msg(self.skt)

    def connect(self, bng, server, port):
        """
        Establishes socket communication with the corresponding vehicle in the
        simulation and calls the connect-hooks on the vehicle's sensors.

        Args:
            bng (:class:`.BeamNGpy`): The running BeamNGpy instance to connect
                                      with.
            server (:class:`socket`): The server socket opened for the vehicle.
        """
        self.bng = bng
        self.server = server
        self.port = port
        self.server.settimeout(60)
        self.skt, addr = self.server.accept()
        self.skt.settimeout(60)

        for name, sensor in self.sensors.items():
            sensor.connect(bng, self)

    def disconnect(self):
        """
        Closes socket communication with the corresponding vehicle.

        Args:
            bng (:class:`.BeamNGpy`): The running BeamNGpy instance to
                                      disconnect from.
        """
        for name, sensor in self.sensors.items():
            sensor.disconnect(self.bng, self)

        self.server.close()
        self.skt.close()
        self.bng = None
        self.server = None
        self.port = None
        self.skt = None
        self.state = None

    def attach_sensor(self, name, sensor):
        """
        Enters a sensor into this vehicle's map of known sensors and calls the
        attach-hook of said sensor. The sensor is identified using the given
        name, which has to be unique among the other sensors of the vehicle.

        Args:
            name (str): The name of the sensor.
            sensor (:class:`beamngpy.Sensor`): The sensor to attach to the
                                               vehicle.
        """
        self.sensors[name] = sensor
        sensor.attach(self, name)

    def detach_sensor(self, name):
        """
        Detaches a sensor from the vehicle's map of known sensors and calls the
        detach-hook of said sensor.

        Args:
            name (str): The name of the sensor to disconnect.
        """
        del self.sensors[name]
        sensor.detach(self, name)

    def encode_sensor_requests(self):
        """
        Encodes engine and vehicle requests for this vehicle's sensors and
        returns them as a tuple of (engine requests, vehicle requests).

        Returns:
            A tuple of two lists: the engine requests and the vehicle requests
            to send to the simulation.
        """
        engine_reqs = dict()
        vehicle_reqs = dict()

        for name, sensor in self.sensors.items():
            engine_req = sensor.encode_engine_request()
            vehicle_req = sensor.encode_vehicle_request()

            if engine_req:
                engine_req['vehicle'] = self.vid
                engine_reqs[name] = engine_req
            if vehicle_req:
                vehicle_reqs[name] = vehicle_req

        engine_reqs = dict(type='SensorRequest', sensors=engine_reqs)
        vehicle_reqs = dict(type='SensorRequest', sensors=vehicle_reqs)
        return engine_reqs, vehicle_reqs

    def decode_sensor_response(self, sensor_data):
        """
        Goes over the given map of sensor data and decodes each of them iff
        they have a corresponding sensor to handle the data in this vehicle.
        The given map of sensor data is expected to have an entries that match
        up with sensor names in this vehicle.

        Args:
            sensor_data (dict): The sensor data to decode as a dictionary,
                                identifying which sensor to decode data with by
                                the name it is known under in this vehicle.

        Returns:
            The decoded data as a dictionary with entries for each sensor name
            and corresponding decoded data.
        """
        response = dict()
        for name, data in sensor_data.items():
            sensor = self.sensors[name]
            data = sensor.decode_response(data)
            response[name] = data
        return response

    def get_engine_flags(self):
        """
        Gathers the engine flag of every sensor known to this vehicle and
        returns them as one dictionary.
        """
        flags = dict()
        for name, sensor in self.sensors.items():
            sensor_flags = sensor.get_engine_flags()
            flags.update(sensor_flags)
        return flags

    def update_vehicle(self):
        """
        Synchronises the :attr:`.Vehicle.state` field with the simulation.
        """
        data = dict(type='UpdateVehicle')
        self.send(data)
        resp = self.recv()
        self.state = resp['state']

    def poll_sensors(self, requests):
        """
        Sends a sensor request to the corresponding vehicle in the simulation
        and returns the raw response data as a dictionary.

        Note:
            This method automatically synchronises the
            :attr:`.Vehicle.state` field with the simulation.
        """
        self.send(requests)
        response = self.recv()
        assert response['type'] == 'SensorData'
        sensor_data = response['data']
        self.state = response['state']
        return sensor_data

    @ack('ShiftModeSet')
    def set_shift_mode(self, mode):
        """
        Sets the shifting mode of the vehicle. This changes whether or not and
        how the vehicle shifts gears depending on the RPM. Available modes are:

         * ``realistic_manual``: Gears have to be shifted manually by the
                                 user, including engaging the clutch.
         * ``realistic_manual_auto_clutch``: Gears have to be shifted manually
                                             by the user, without having to
                                             use the clutch.
         * ``arcade``: Gears shift up and down automatically. If the brake is
                       held, the vehicle automatically shifts into reverse
                       and accelerates backward until brake is released or
                       throttle is engaged.
         * ``realistic_automatic``: Gears shift up automatically, but reverse
                                    and parking need to be shifted to
                                    manually.

        Args:
            mode (str): The mode to set. Must be a string from the options
                        listed above.

        Raises:
            BNGValueError: If an invalid mode is given.
        """
        mode = mode.lower().strip()
        if mode not in SHIFT_MODES:
            raise BNGValueError('Non-existent shift mode: {}'.format(mode))

        mode = SHIFT_MODES[mode]
        data = dict(type='SetShiftMode')
        data['mode'] = mode
        self.send(data)

    @ack('Controlled')
    def control(self, **options):
        """
        Sends a control message to the vehicle, setting vehicle inputs
        accordingly. Possible values to set are:

         * ``steering``: Rotation of the steering wheel, from -1.0 to 1.0.
         * ``throttle``: Intensity of the throttle, from 0.0 to 1.0.
         * ``brake``: Intensity of the brake, from 0.0 to 1.0.
         * ``parkingbrake``: Intensity of the parkingbrake, from 0.0 to 1.0.
         * ``clutch``: Clutch level, from 0.0 to 1.0.
         * ``gear``: Gear to shift to

        Args:
            **kwargs (dict): The input values to set.
        """
        data = dict(type='Control', **options)
        self.send(data)

    @ack('AiModeSet')
    def ai_set_mode(self, mode):
        """
        Sets the desired mode of the simulator's built-in AI for this vehicle.
        Possible values are:

         * ``disabled``: Turn the AI off (default state)
         * ``random``: Drive from random points to random points on the map
         * ``span``: Drive along the entire road network of the map
         * ``manual``: Drive to a specific waypoint, target set separately
         * ``chase``: Chase a target vehicle, target set separately
         * ``flee``: Flee from a vehicle, target set separately
         * ``stopping``: Make the vehicle come to a halt (AI disables itself
                                                            once the vehicle
                                                            stopped.)

        Note:
            Some AI methods automatically set appropriate modes, meaning a call
            to this method might be optional.

        Args:
            mode (str): The AI mode to set.
        """
        data = dict(type='SetAiMode')
        data['mode'] = mode
        self.send(data)

    @ack('AiSpeedSet')
    def ai_set_speed(self, speed, mode='limit'):
        """
        Sets the target speed for the AI in m/s. Speed can be maintained in two
        modes:

         * ``limit``: Drive speeds between 0 and the limit, as the AI
                        sees fit.
         * ``set``: Try to maintain the given speed at all times.

        Args:
            speed (float): The target speed in m/s.
            mode (str): The speed mode.
        """
        data = dict(type='SetAiSpeed')
        data['speed'] = speed
        data['mode'] = mode
        self.send(data)

    @ack('AiTargetSet')
    def ai_set_target(self, target, mode='chase'):
        """
        Sets the target to chase or flee. The target should be the ID of
        another vehicle in the simulation. The AI is automatically set to the
        given mode.

        Args:
            target (str): ID of the target vehicle as a string.
            mode(str): How the target should be treated. `chase` to chase the
                       target, `flee` to flee from it.
        """
        self.ai_set_mode(mode)
        data = dict(type='SetAiTarget')
        data['target'] = target
        self.send(data)

    @ack('AiWaypointSet')
    def ai_set_waypoint(self, waypoint):
        """
        Sets the waypoint the AI should drive to in manual mode. The AI gets
        automatically set to manual mode when this method is called.

        Args:
            waypoint (str): ID of the target waypoint as a string.
        """
        self.ai_set_mode('manual')
        data = dict(type='SetAiWaypoint')
        data['target'] = waypoint
        self.send(data)

    @ack('AiDriveInLaneSet')
    def ai_drive_in_lane(self, lane):
        """
        Sets the drive in lane flag of the AI. If True, the AI only drives
        within the lane it can legally drive in.

        Args:
            lane (bool): Lane flag to set.
        """
        data = dict(type='SetDriveInLane')
        if lane:
            lane = 'on'
        else:
            lane = 'off'
        data['lane'] = lane
        self.send(data)

    @ack('AiLineSet')
    def ai_set_line(self, line, cling=True):
        """
        Makes the AI follow a given polyline. The line is specified as a list
        of dictionaries where each dictionary has a `pos` entry specifying the
        supposed position as an (x, y, z) triplet and a `speed` entry
        specifying the speed in m/s.

        Args:
            line (list): Polyline as list of dicts as described above.
            cling (bool): Whether or not to align the z coordinate of
        """
        data = dict(type='SetAiLine')
        data['line'] = line
        data['cling'] = cling
        self.send(data)

    @ack('AiScriptSet')
    def ai_set_script(self, script, start_dir=None, up_dir=None, cling=True):
        """
        Makes the vehicle follow a given "script" -- a script being a list of
        timestamped positions defining where a vehicle should be at what time.
        This can be used to make the vehicle drive a long a polyline with speed
        implicitly expressed in the time between points.

        Args:
            script (list): A list of nodes in the script. Each node is expected
                           to be a dict-like that has `x`, `y`, and `z`
                           entries for the supposed position of the vehicle,
                           and a `t` entry for the time of the node along the
                           path. Time values are in seconds relative to the
                           time when script playback is started.
            start_dir (tuple): Optional initial directional vector of the
                               vehicle. If not specified, it's inferred from
                               the first two points.
            up_dir (tuple): Optional initial up vector of the vehicle. It can
                            be set alongside `start_dir` to change orientation
                            of the vehicle at the start. Otherwise, (0, 0, 1)
                            is assumed as default.
            cling (bool): A flag that makes the simulator cling z-coordinates
                          to the ground. Since computing z-coordinates in
                          advance without knowing the level geometry can be
                          cumbersome, this flag is used to automatically set
                          z-coordinates in the script to the ground height.
                          Defaults to True.

        Raises:
            BNGValueError: If the script has fewer than three nodes, the
                           minimum length of a script.
        """
        if len(script) < 3:
            raise BNGValueError('AI script must have at least 3 nodes.')

        if not start_dir:
            fst = script[0]
            snd = script[1]
            start_dir = (
                snd['x'] - fst['x'],
                snd['y'] - fst['y'],
                snd['z'] - fst['z'],
            )
            up_dir = (0, 0, 1)

        script[0]['dir'] = start_dir
        script[0]['up'] = up_dir

        data = dict(type='SetAiScript')
        data['script'] = script
        data['cling'] = cling
        self.send(data)

    @ack('AiAggressionSet')
    def ai_set_aggression(self, aggr):
        data = dict(type='SetAiAggression')
        data['aggression'] = aggr
        self.send(data)

    def get_part_options(self):
        """
        Retrieves a tree of part configuration options for this vehicle.

        Returns:
            A tree of part configuration options for this vehicle expressed
            as nested dictionaries.
        """
        data = dict(type='GetPartOptions')
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'PartOptions'
        return resp['options']

    def get_part_config(self):
        """
        Retrieves the current part configuration of this vehicle. The
        configuration contains both the current values of adjustable vehicle
        parameters and a mapping of part types to their currently-selected
        part.

        Returns:
            The current vehicle configuration as a dictionary.
        """
        data = dict(type='GetPartConfig')
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'PartConfig'
        resp = resp['config']
        if not resp['parts']:
            resp['parts'] = dict()
        if not resp['vars']:
            resp['vars'] = dict()
        return resp

    def set_part_config(self, cfg):
        """
        Sets the current part configuration of this vehicle. The configuration
        is given as a dictionary containing both adjustable vehicle parameters
        and a mapping of part types to their selected parts.

        Args:
            cfg (dict): The new vehicle configuration as a dictionary.

        Notes:
            Changing parts causes the vehicle to respawn, which repairs it as
            a side-effect.
        """
        data = dict(type='SetPartConfig')
        data['config'] = cfg
        data['vid'] = self.vid
        self.send(data)
        self.bng.await_vehicle_spawn(self.vid)
        self.skt.close()
        self.server.close()
        self.skt = None
        self.bng.connect_vehicle(self, self.port)

    @ack('ColorSet')
    def set_color(self, rgba=(1., 1., 1., 1.)):
        """
        Sets the colour of this vehicle. Colour can be adjusted on the RGB
        spectrum and the "shininess" of the paint.

        Args:
            rgba (tuple): The new colour given as a tuple of RGBA floats, where
                          the alpha channel encodes the shininess of the paint.
        """
        data = dict(type='SetColor')
        data['r'] = rgba[0]
        data['g'] = rgba[1]
        data['b'] = rgba[2]
        data['a'] = rgba[3]
        self.send(data)

    set_colour = set_color

    def get_bbox(self):
        """
        Returns this vehicle's current bounding box as a dictionary containing
        eight points.

        Returns:
            The vehicle's current bounding box as a dictionary of eight points.
            Points are named following the convention that the cuboid has a
            "near" rectangle towards the rear of the vehicle and "far"
            rectangle towards the front. The points are then named like this:

            * `near_bottom_left`: Bottom left point of the near rectangle as an
                                  (x, y ,z) triplet
            * `near_bottom_right`: Bottom right point of the near rectangle as
                                   an (x, y, z) triplet
            * `near_top_left`: Top left point of the near rectangle as an
                               (x, y, z) triplet
            * `near_top_right`: Top right point of the near rectangle as an
                                (x, y, z) triplet
            * `far_bottom_left`: Bottom left point of the far rectangle as an
                                 (x, y, z) triplet
            * `far_bottom_right`: Bottom right point of the far rectangle as an
                                  (x, y, z) triplet
            * `far_top_left`: Top left point of the far rectangle as an
                              (x, y, z) triplet
            * `far_top_right`: Top right point of the far rectangle as an
                               (x, y, z) triplet
        """
        if self.bng is None:
            raise BNGError('The vehicle needs to be loaded in the simulator '
                           'to obtain its current bounding box.')
        return self.bng.get_vehicle_bbox(self)

    def annotate_parts(self):
        """
        Triggers the process to have individual parts of a vehicle have unique
        annotation colors.
        """
        self.bng.annotate_parts(self)

    def revert_annotations(self):
        """
        Reverts per-part annotations of this vehicle such that it will be
        annotated with the same color for the entire vehicle.
        """
        self.bng.revert_annotations(self)

    def close(self):
        """
        Closes this vehicle's and its sensors' connection and detaches all
        sensors.
        """
        self.disconnect()

        for name, sensor in self.sensors.items():
            sensor.detach(self, name)

        self.sensors = dict()
