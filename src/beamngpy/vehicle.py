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

    def __init__(self, vid, **options):
        self.vid = vid

        self.host = None
        self.port = None

        self.server = None
        self.skt = None

        self.sensors = dict()

        self.options = options

        self.sensor_cache = dict()

    def send(self, data):
        return send_msg(self.skt, data)

    def recv(self):
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
        self.server = server
        self.skt, addr = self.server.accept()

        for name, sensor in self.sensors.items():
            sensor.connect(bng, self)

    def disconnect(self, bng):
        pass

    def attach_sensor(self, name, sensor):
        self.sensors[name] = sensor
        sensor.attach(self, name)

    def detach_sensor(self, name):
        del self.sensors[name]
        sensor.detach(self, name)

    def encode_sensor_requests(self):
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
        response = dict()
        for name, data in sensor_data.items():
            sensor = self.sensors[name]
            data = sensor.decode_response(data)
            response[name] = data
        return response

    def get_engine_flags(self):
        flags = dict()
        for name, sensor in self.sensors.items():
            sensor_flags = sensor.get_engine_flags()
            flags.update(sensor_flags)
        return flags

    def poll_sensors(self, requests):
        self.send(requests)
        response = self.recv()
        assert response['type'] == 'SensorData'
        sensor_data = response['data']
        return sensor_data

    @ack('Controlled')
    def control(self, **options):
        data = dict(type='Control', **options)
        self.send(data)
