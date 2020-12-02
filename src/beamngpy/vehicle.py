"""
.. module:: vehicle
    :platform: Windows
    :synopsis: Contains vehicle-related classes/functions  for BeamNGpy.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>

"""

import base64
import logging as log
import warnings

from .beamngcommon import *
from .sensors import State

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

    def __init__(self, vid, model, **options):
        """
        Creates a vehicle with the given vehicle ID. The ID must be unique
        within the scenario.

        Args:
            vid (str): The vehicle's ID.
            model (str): Model of the vehicle.
        """
        self.vid = vid.replace(' ', '_')

        self.port = None

        self.bng = None
        self.server = None
        self.skt = None

        self.sensors = dict()

        options['model'] = model
        options['licenseText'] = options.get('licence')
        options['color'] = options.get('colour', options.get('color'))
        options['color2'] = options.get('colour2', options.get('color2'))
        options['color3'] = options.get('colour3', options.get('color3'))
        self.options = options

        self.sensor_cache = dict()

        self.extensions = options.get('extensions')

        self._veh_state_sensor_id = "state"
        state = State()
        self.attach_sensor(self._veh_state_sensor_id, state)
        

    def __hash__(self):
        return hash(self.vid)

    def __eq__(self, other):
        if isinstance(other, type(self)):
            return self.vid == other.vid
        return False

    def __str__(self):
        return 'V:{}'.format(self.vid)

    @property
    def state(self):
        """
        This property contains the vehicle's current state in the running
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
        return self.sensors[self._veh_state_sensor_id].data

    @state.setter
    def state(self, value):
        self.sensors[self._veh_state_sensor_id].data = value

    @state.deleter
    def state(self):
        del self.sensors[self._veh_state_sensor_id].data

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
        self.skt, _ = self.server.accept()
        self.skt.settimeout(60)

        for _, sensor in self.sensors.items():
            sensor.connect(bng, self)

    def disconnect(self):
        """
        Closes socket communication with the corresponding vehicle.

        Args:
            bng (:class:`.BeamNGpy`): The running BeamNGpy instance to
                                      disconnect from.
        """
        for _, sensor in self.sensors.items():
            sensor.disconnect(self.bng, self)

        self.server.close()
        self.skt.close()
        self.bng = None
        self.server = None
        self.port = None
        self.skt = None

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
        if name in self.sensors:
            sensor = self.sensors[name]
            sensor.detach(self, name)
        del self.sensors[name]

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
        warnings.warn("update_vehicle is deprecated\nthe .Vehicle.state attribute is now a default sensor for every vehicle and is updated through poll_sensors", DeprecationWarning)
        return self.state

    def poll_sensors(self, requests=None):
        """
        Updates the vehicle's sensor readings.
        Args:
            mode (str): The mode to set. Must be a string from the options
                        listed above.

        Raises:
            DeprecationWarning: If requests parameter is used.
            DeprecationWarning: Always, since the return type will change in the future.
        
        Returns:
            Dict with sensor data to support compatibility with previous versions.
        """
        if requests!=None:
            warnings.warn("do not use 'requests' as function argument\nit is not used and will be removed in future versions", DeprecationWarning)
        warnings.warn("return type will be None in future versions", DeprecationWarning)

        engine_reqs, vehicle_reqs = self.encode_sensor_requests()
        sensor_data = dict()
        compatibility_support = None

        if engine_reqs['sensors']:
            self.bng.send(engine_reqs)
            response = self.bng.recv()
            assert response['type'] == 'SensorData'
            sensor_data.update(response['data'])

        if vehicle_reqs['sensors']:
            self.send(vehicle_reqs)
            response = self.recv()
            assert response['type'] == 'SensorData'
            compatibility_support = response['data']
            sensor_data.update(response['data'])

        result = self.decode_sensor_response(sensor_data)
        for sensor, data in result.items():
            self.sensors[sensor].data = data

        self.sensor_cache = result
        return compatibility_support

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
    def ai_set_script(self, script, start_dir=None, up_dir=None, cling=True,
                      teleport=True):
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
            cling (bool): A flag that makes the simulator cling z-coordinates
                          to the ground. Since computing z-coordinates in
                          advance without knowing the level geometry can be
                          cumbersome, this flag is used to automatically set
                          z-coordinates in the script to the ground height.
                          Defaults to True.

        Notes:
            The AI follows the given script the best it can. It cannot drive
            along scripts that would be physically impossible, e.g. specifying
            a script with points A & B one kilometer apart and giving it a
            a second between those points will make the AI drive from A to B as
            fast as it can, but unlikely to reach it in the given time.
            Furthermore, if the AI falls behind schedule, it will start
            skipping points in the script in an effort to make up for
            lost time.

        Raises:
            BNGValueError: If the script has fewer than three nodes, the
                           minimum length of a script.
        """
        if start_dir != None or up_dir != None or teleport != None:
            warnings.warn("The function arguments 'start_dir', 'up_dir', and 'teleport' are not used anymore and will be removed in future versions.", DeprecationWarning) 
        if len(script) < 3:
            raise BNGValueError('AI script must have at least 3 nodes.')

        data = dict(type='SetAiScript')
        data['script'] = script
        data['cling'] = cling
        self.send(data)

    @ack('AiAggressionSet')
    def ai_set_aggression(self, aggr):
        data = dict(type='SetAiAggression')
        data['aggression'] = aggr
        self.send(data)

    @ack('ExecutedLuaChunkVE')
    def queue_lua_command(self, chunk):
        """
        Executes lua chunk in the vehicle engine VM.

        Args:
            chunk(str): lua chunk as a string
        """
        data = dict(type='QueueLuaCommandVE')
        data['chunk'] = chunk
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
        Sets the color of this vehicle. Colour can be adjusted on the RGB
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
        if self.bng is None:
            raise BNGError('The vehicle needs to be loaded in the simulator '
                           'to obtain its current bounding box.')
        return self.bng.get_vehicle_bbox(self)

    @ack('LightsSet')
    def set_lights(self, **kwargs):
        """
        Sets the vehicle's lights to given intensity values. The lighting
        system features lights that are simply binary on/off, but also ones
        where the intensity can be varied. Binary lights include:

            * `left_signal`
            * `right_signal`
            * `hazard_signal`

        Non-binary lights vary between 0 for off, 1 for on, 2 for higher
        intensity. For example, headlights can be turned on with 1 and set to
        be more intense with 2. Non-binary lights include:

            * `headlights`
            * `fog_lights`
            * `lightbar`

        Args:
            left_signal (bool): On/off state of the left signal
            right_signal (bool): On/off state of the right signal
            hazard_signal (bool): On/off state of the hazard lights
            headlights (int): Value from 0 to 2 indicating headlight intensity
            fog_lights (int): Value from 0 to 2 indicating fog light intensity
            lightbar (int): Value from 0 to 2 indicating lightbar intensity

        Note:
            Not every vehicle has every type of light. For example, the
            `lightbar` refers to the kind of lights typically found on top of
            police cars. Setting values for non-existent lights will not cause
            an error, but also achieve no effect.

            Note also that lights are not independent. For example, turning on
            the hazard lights will make both signal indicators blink, meaning
            they will be turned on as well. Opposing indicators also turn each
            other off, i.e. turning on the left signal turns off the right one,
            and turning on the left signal during

        Raises:
            BNGValueError: If an invalid light value is given.

        Returns:
            Nothing. To query light states, attach an
            :class:`.sensors.Electrics` sensor and poll it.
        """
        lights = {}

        left_signal = kwargs.get('left_signal', None)
        if left_signal is not None:
            if not isinstance(left_signal, bool):
                raise BNGValueError('Non-boolean value for left_signal.')
            lights['leftSignal'] = left_signal

        right_signal = kwargs.get('right_signal', None)
        if right_signal is not None:
            if not isinstance(right_signal, bool):
                raise BNGValueError('Non-boolean value for right_signal.')
            lights['rightSignal'] = right_signal

        hazard_signal = kwargs.get('hazard_signal', None)
        if hazard_signal is not None:
            if not isinstance(hazard_signal, bool):
                raise BNGValueError('Non-boolean value for hazard_signal.')
            lights['hazardSignal'] = hazard_signal

        valid_lights = {0, 1, 2}

        headlights = kwargs.get('headlights', None)
        if headlights is not None:
            if not isinstance(headlights, int):
                raise BNGValueError('Non-int value given for headlights.')
            if headlights not in valid_lights:
                msg = 'Invalid value given for headlights, must be ' \
                      '0, 1, or 2, but was: ' + str(headlights)
                raise BNGValueError(msg)
            lights['headLights'] = headlights

        fog_lights = kwargs.get('fog_lights', None)
        if fog_lights is not None:
            if not isinstance(fog_lights, int):
                raise BNGValueError('Non-int value given for fog lights.')
            if fog_lights not in valid_lights:
                msg = 'Invalid value given for fog lights, must be ' \
                      '0, 1, or 2, but was: ' + str(fog_lights)
                raise BNGValueError(msg)
            lights['fogLights'] = fog_lights

        lightbar = kwargs.get('lightbar', None)
        if lightbar is not None:
            if not isinstance(lightbar, int):
                raise BNGValueError('Non-int value given for lighbar.')
            if lightbar not in valid_lights:
                msg = 'Invalid value given for lightbar, must be ' \
                      '0, 1, or 2, but was: ' + str(lightbar)
                raise BNGValueError(msg)
            lights['lightBar'] = lightbar

        lights['type'] = 'SetLights'
        self.send(lights)

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
