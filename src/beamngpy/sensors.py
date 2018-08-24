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


class Sensor:

    def attach(self, vehicle, name):
        raise NotImplementedError('Subclasses have to implement this.')

    def detach(self, vehicle, name):
        raise NotImplementedError('Subclasses have to implement this.')

    def encode_engine_request(self):
        raise NotImplementedError('Subclasses have to implement this.')

    def encode_vehicle_request(self):
        raise NotImplementedError('Subclasses have to implement this.')

    def decode_response(self, resp):
        raise NotImplementedError('Subclasses have to implement this.')


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

    def encode_vehicle_request(self):
        return None

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


class Lidar(Sensor):
    pass
