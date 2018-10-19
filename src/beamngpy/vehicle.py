"""
.. module:: vehicle
    :platform: Windows
    :synopsis: Contains vehicle-related classes/functions  for BeamNGpy.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""

import base64
import logging as log

from .beamng_common import *


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

        self.host = None
        self.port = None

        self.server = None
        self.skt = None

        self.sensors = dict()

        options['color'] = options.get('colour', options.get('color', None))
        self.options = options

        self.sensor_cache = dict()

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

    def __hash__(self):
        return hash(self.vid)

    def __eq__(self, other):
        if isinstance(other, type(self)):
            return self.vid == other.vid
        return False

    def __str__(self):
        return 'V:{}'.format(self.vid)

    def connect(self, bng, server):
        """
        Establishes socket communication with the corresponding vehicle in the
        simulation and calls the connect-hooks on the vehicle's sensors.

        Args:
            bng (:class:`.BeamNGpy`): The running BeamNGpy instance to connect
                                      with.
            server (:class:`socket`): The server socket opened for the vehicle.
        """
        self.server = server
        self.skt, addr = self.server.accept()

        for name, sensor in self.sensors.items():
            sensor.connect(bng, self)

    def disconnect(self, bng):
        """
        Closes socket communication with the corresponding vehicle.

        Args:
            bng (:class:`.BeamNGpy`): The running BeamNGpy instance to
                                      disconnect from.
        """
        pass

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

    def poll_sensors(self, requests):
        """
        Sends a sensor request to the corresponding vehicle in the simulation
        and returns the raw response data as a dictionary.
        """
        self.send(requests)
        response = self.recv()
        assert response['type'] == 'SensorData'
        sensor_data = response['data']
        return sensor_data

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

        Args:
            **kwargs (dict): The input values to set.
        """
        data = dict(type='Control', **options)
        self.send(data)

    @ack('AiModeSet')
    def ai_set_mode(self, mode):
        data = dict(type='SetAiMode')
        data['mode'] = mode
        self.send(data)

    @ack('AiSpeedSet')
    def ai_set_speed(self, speed, mode='limit'):
        data = dict(type='SetAiSpeed')
        data['speed'] = speed
        data['mode'] = mode
        self.send(data)

    @ack('AiTargetSet')
    def ai_set_target(self, target):
        data = dict(type='SetAiTarget')
        data['target'] = target
        self.send(data)

    @ack('AiSpanSet')
    def ai_set_span(self, span):
        data = dict(type='SetAiSpan')
        data['span'] = span
        self.send(data)

    @ack('AiDriveInLaneSet')
    def ai_drive_in_lane(self, lane):
        data = dict(type='SetDriveInLane')
        data['lane'] = lane
        self.send(data)
