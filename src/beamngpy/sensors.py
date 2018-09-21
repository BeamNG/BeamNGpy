"""
.. module:: sensors
    :platform: Windows
    :synopsis: Module containing the various sensors one can attach to a
               vehicle.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""
import base64
import logging as log
import mmap

import numpy as np
from PIL import Image

NEAR = 0.01
FAR = 300

LIDAR_POINTS = 500000


class Sensor:

    def attach(self, vehicle, name):
        pass

    def detach(self, vehicle, name):
        pass

    def encode_engine_request(self):
        return None

    def encode_vehicle_request(self):
        return None

    def decode_response(self, resp):
        return resp

    def connect(self, bng, vehicle):
        pass

    def disconnect(self, bng, vehicle):
        pass

    def get_engine_flags(self):
        return dict()


class Camera(Sensor):

    def __init__(self, pos, direction, fov, resolution, near_far=(NEAR, FAR),
                 colour=False, depth=False, annotation=False):
        self.pos = pos
        self.direction = direction
        self.fov = fov
        self.resolution = resolution
        self.near_far = near_far

        self.colour = colour
        self.depth = depth
        self.annotation = annotation

        self.colour_handle = None
        self.colour_shmem = None
        self.depth_handle = None
        self.depth_shmem = None
        self.annotation_handle = None
        self.annotation_shmem = None

    def attach(self, vehicle, name):
        # TODO: Place PID in shmem handle for OS-wide uniqueness
        size = self.resolution[0] * self.resolution[1] * 4  # RGBA / L are 4bbp
        if self.colour:
            self.colour_handle = '{}.{}.colour'.format(vehicle.vid, name)
            self.colour_shmem = mmap.mmap(0, size, self.colour_handle)
            log.debug('Bound memory for colour: %s', self.colour_handle)

        if self.depth:
            self.depth_handle = '{}.{}.depth'.format(vehicle.vid, name)
            self.depth_shmem = mmap.mmap(0, size, self.depth_handle)
            log.debug('Bound memory for depth: %s', self.depth_handle)

        if self.annotation:
            self.annotation_handle = '{}.{}.annotate'.format(vehicle.vid, name)
            self.annotation_shmem = mmap.mmap(0, size, self.annotation_handle)
            log.debug('Bound memory for annotation: %s',
                      self.annotation_handle)

    def detach(self, vehicle, name):
        if self.colour_shmem:
            log.debug('Unbinding memory for color: %s', self.colour_handle)
            self.colour_shmem.close()

        if self.depth_shmem:
            log.debug('Unbinding memory for depth: %s', self.depth_handle)
            self.depth_shmem.close()

        if self.annotation_shmem:
            log.debug('Unbinding memory for annotation: %s',
                      self.annotation_handle)
            self.annotation_shmem.close()

    def connect(self, bng, vehicle):
        size = self.resolution[0] * self.resolution[1] * 4  # RGBA / L are 4bbp

        if self.colour_shmem:
            bng.open_shmem(self.colour_handle, size)

        if self.depth_shmem:
            bng.open_shmem(self.depth_handle, size)

        if self.annotation_shmem:
            bng.open_shmem(self.annotation_handle, size)

    def disconnect(self, bng, vehicle):
        if self.colour_shmem:
            bng.close_shmem(self.colour_handle)

        if self.depth_shmem:
            bng.close_shmem(self.depth_handle)

        if self.annotation_shmem:
            bng.close_shmem(self.annotation_handle)

    def encode_engine_request(self):
        req = dict(type='Camera')

        if self.colour:
            req['color'] = self.colour_handle

        if self.depth:
            req['depth'] = self.depth_handle

        if self.annotation:
            req['annotation'] = self.annotation_handle

        req['pos'] = self.pos
        req['direction'] = self.direction
        req['fov'] = self.fov
        req['resolution'] = self.resolution
        req['near_far'] = self.near_far

        return req

    def decode_response(self, resp):
        decoded = dict(type='Camera')
        img_w = resp['width']
        img_h = resp['height']

        size = img_w * img_h * 4

        if self.colour:
            self.colour_shmem.seek(0)
            colour_d = self.colour_shmem.read(size)
            colour_d = np.frombuffer(colour_d, dtype=np.uint8)
            colour_d = colour_d.reshape(img_h, img_w, 4)
            decoded['colour'] = Image.fromarray(colour_d)

        if self.annotation:
            self.annotation_shmem.seek(0)
            annotate_d = self.annotation_shmem.read(size)
            annotate_d = np.frombuffer(annotate_d, dtype=np.uint8)
            annotate_d = annotate_d.reshape(img_h, img_w, 4)
            decoded['annotation'] = Image.fromarray(annotate_d)

        if self.depth:
            self.depth_shmem.seek(0)
            depth_d = self.depth_shmem.read(size)
            depth_d = np.frombuffer(depth_d, dtype=np.float32)
            depth_d = depth_d / FAR
            depth_d = depth_d.reshape(img_h, img_w)
            depth_d = np.uint8(depth_d * 255)
            decoded['depth'] = Image.fromarray(depth_d)

        return decoded

    def get_engine_flags(self):
        flags = dict()
        if self.annotation_shmem:
            flags['annotations'] = True
        return flags


class Lidar(Sensor):
    shmem_size = LIDAR_POINTS * 3 * 4

    def __init__(self):
        self.handle = None
        self.shmem = None

    def attach(self, vehicle, name):
        self.handle = '{}.{}.lidar'.format(vehicle.vid, name)
        self.shmem = mmap.mmap(0, Lidar.shmem_size, self.handle)
        log.debug('Bound memory for lidar: %s', self.handle)

    def detach(self, vehicle, name):
        self.shmem.close()

    def encode_engine_request(self):
        req = dict(type='Lidar')
        req['shmem'] = self.handle
        req['size'] = Lidar.shmem_size
        # TODO: Make Lidar customisable
        return req

    def decode_response(self, resp):
        size = resp['size']
        self.shmem.seek(0)
        points_buf = self.shmem.read(size)
        points_buf = np.frombuffer(points_buf, dtype=np.float32)
        assert points_buf.size % 3 == 0
        points_buf = points_buf.reshape(points_buf.size // 3, 3)
        return resp

    def get_engine_flags(self):
        flags = dict(lidar=True)
        return flags


class GForces(Sensor):

    def attach(self, vehicle, name):
        log.debug('Attaching GForces sensor %s to: %s', name, vehicle.vid)

    def detach(self, vehicle, name):
        log.debug('Detaching GForces sensor %s from: %s', name, vehicle.vid)

    def encode_vehicle_request(self):
        req = dict(type='GForces')
        return req


class Electrics(Sensor):

    def attach(self, vehicle, name):
        log.debug('Attaching Electrics sensor %s to: %s', name, vehicle.vid)

    def detach(self, vehicle, name):
        log.debug('Detaching Electrics sensor %s from: %s', name, vehicle.vid)

    def encode_vehicle_request(self):
        req = dict(type='Electrics')
        return req


class Damage(Sensor):

    def attach(self, vehicle, name):
        log.debug('Attaching Damage sensor %s to: %s', name, vehicle.vid)

    def detach(self, vehicle, name):
        log.debug('Detaching Damage sensor %s to: %s', name, vehicle.vid)

    def encode_vehicle_request(self):
        req = dict(type='Damage')
        return req
