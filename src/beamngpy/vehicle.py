"""
.. module:: vehicle
    :platform: Windows
    :synopsis: Contains vehicle-related classes/functions  for BeamNGpy.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""

import base64
import logging as log


class Vehicle:

    def __init__(self, vid):
        self.vid = vid

        self.host = None
        self.port = None

        self.server = None
        self.skt = None

        self.sensors = dict()

    def __hash__(self):
        return hash(self.vid)

    def __eq__(self, other):
        if isinstance(other, type(self)):
            return self.vid == other.vid
        return False

    def __str__(self):
        return 'V:{}'.format(self.vid)

    def setup(self, server):
        self.server = server
        self.skt = self.server.accept()

    def attach_sensor(self, name, sensor):
        pass

    def detach_sensor(self, name):
        pass

    def poll_sensors(self):
        pass

    def vcontrol(self, inputs):
        pass
