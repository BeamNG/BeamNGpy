"""
.. module:: sensors
    :platform: Windows
    :synopsis: Module containing the various sensors one can attach to a
               vehicle.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

This module implements various sensors that can be attached to vehicles to
extract data from simulations.
"""
import base64
import logging as log
import mmap
import os
import sys

import numpy as np
from PIL import Image

NEAR = 0.01
FAR = 1000

LIDAR_POINTS = 2000000


class Sensor:
    """
    Sensor meta-class declaring methods common to them.
    """

    def attach(self, vehicle, name):
        """
        Called when the sensor is attached to a :class:`.Vehicle` instance.
        Used to perform sensor setup code before the simulation is started.
        This is called *after* the sensor has been entered into the vehicle's
        map of sensors under the given name.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle instance the sensor is
                                         being attached to.
            name (str): The name the sensor is known under to the vehicle.
        """
        pass

    def detach(self, vehicle, name):
        """
        Called when the sensor is detached from a :class:`.Vehicle` instance.
        Used to perform sensor teardown code after the simulation is finished.
        This is called *after* the sensor has been removed from the
        vehicle's map of sensors under the given name.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle instance the sensor is
                                         being detached from.
            name (str): The name the sensor was known under to the vehicle.
        """
        pass

    def encode_engine_request(self):
        """
        Called to retrieve this sensor's data request to the engine as a
        dictionary. The dictionary returned by this method will be bundled
        along the vehicle's other sensors' requests as a SensorRequest to the
        game's engine.

        Note:
            Sensors require corresponding code in the simulator to handle
            requests.

        Example:
            Consult the implementation of the :class:`.Camera` sensor for a
            good example of an engine request.

        Returns:
            The request to send to the engine as a dictionary.
        """
        return None

    def encode_vehicle_request(self):
        """
        Called to retrieve this sensor's request to the vehicle as a
        dictionary. The dictionary returned by this method will be bundled
        along the vehicle's other sensors' requests as a SensorRequest to the
        attached vehicle.

        Note:
            Sensors require corresponding code in the simulator to handle
            requests.

        Example:
            Consult the implementation of the :class:`.Electrics` sensor for a
            good example of a vehicle request.

        Returns:
            The request to send to the vehicle as a dictionary.
        """
        return None

    def decode_response(self, resp):
        """
        Called to do post-processing on sensor data obtained from the
        simulation. This method is called after raw simulation data is received
        and the resulting processed data is considered the result of a sensor
        request.

        Example:
            Consult the implementation of the :class:`.Camera` sensor for a
            good example of decoding sensor data.
        """
        return resp

    def connect(self, bng, vehicle):
        """
        Called when the attached vehicle is being initialised in the
        simulation. This method is used to perform setup code that requires the
        simulation to be running.
        """
        pass

    def disconnect(self, bng, vehicle):
        """
        Called when the attached vehicle is being removed from simulation. This
        method is used to perform teardown code after the simulation.
        """
        pass

    def get_engine_flags(self):
        """
        Called to retrieve a dictionary of settings in the engine this sensor
        requires.

        Returns:
            A dictionary of flags to set in the engine for this sensor to
            function.
        """
        return dict()


class Camera(Sensor):
    """
    A camera sensor provides several types of image data from a user-defined
    perspective relative to the vehicle. It can provide the following types of
    data:

    * Colour images
    * Pixel-wise depth
    * Pixel-wise object annotation

    A single camera sensor can be configured to provide any or all of these
    data at once, ensuring they all align to the same perspective.
    """

    def __init__(self, pos, direction, fov, resolution, near_far=(NEAR, FAR),
                 colour=False, depth=False, annotation=False):
        """
        The camera sensor is set up with a fixed offset position and
        directional vector to face relative to the vehicle. This means as the
        vehicle moves and rotates, the camera is moved and rotated accordingly.

        Besides position and orientation, the image can further be customised
        with the FoV angle the camera should have, the resolution of the
        image(s) it outputs, and the near/far plane at which objects get
        clipped from view.

        Which sensor data to provide can be indicated using boolean flags for
        the corresponding type.


        Args:
            pos (tuple): (x,y,z) tuple of the camera's position offset relative
                         to the vehicle it's attached to.
            direction (tuple): (x,y,z) tuple expressing the direction vector
                               the camera is facing.
            fov (float): The Field of View of the camera.
            resolution (tuple): (width,height) tuple encoding the camera's
                                output resolution.
            near_far (tuple): (near,far) tuple of the distance below which and
                              after which geometry gets clipped. Usually
                              does not need to be changed.
            colour (bool): Whether to output colour information.
            depth (bool): Whether to output depth information.
            annotation (bool): Whether to output annotation information.
        """
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
        """
        This method is called when the camera is attached and allocates
        shared memory space to house the sensor data the camera is supposed
        to provide.


        Args:
            vehicle (:class:`.Vehicle`): The vehicle the camera is being
                                         attached to.
            name (str): The name of the camera.
        """
        pid = os.getpid()
        prefix = ''
        if vehicle:
            prefix = vehicle.vid
        size = self.resolution[0] * self.resolution[1] * 4  # RGBA / L are 4bbp
        # if self.colour:
        self.colour_handle = '{}.{}.{}.colour'.format(pid, prefix, name)
        self.colour_shmem = mmap.mmap(0, size, self.colour_handle)
        log.debug('Bound memory for colour: %s', self.colour_handle)

        # if self.depth:
        self.depth_handle = '{}.{}.{}.depth'.format(pid, prefix, name)
        self.depth_shmem = mmap.mmap(0, size, self.depth_handle)
        log.debug('Bound memory for depth: %s', self.depth_handle)

        # if self.annotation:
        self.annotation_handle = '{}.{}.{}.annotate'.format(pid, prefix, name)
        self.annotation_shmem = mmap.mmap(0, size, self.annotation_handle)
        log.debug('Bound memory for annotation: %s',
                  self.annotation_handle)

    def detach(self, vehicle, name):
        """
        This method is called when the camera is detached from the vehicle. It
        de-allocates the shared memory space obtained for sensor data.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle the camera is being
                                         detached from.
            name (str): The name of the camera.
        """
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
        """
        This method is called when the vehicle is set up in the simulation.
        It's used to inform the simulation about the shared memory used to
        exchange sensor data for this camera.

        Args:
            bng (:class:`.BeamNGpy`): Running instance of BeamNGpy.
            vehicle (:class:`.Vehicle`): The vehicle being connected.
        """
        size = self.resolution[0] * self.resolution[1] * 4  # RGBA / L are 4bbp

        if self.colour_shmem:
            bng.open_shmem(self.colour_handle, size)

        if self.depth_shmem:
            bng.open_shmem(self.depth_handle, size)

        if self.annotation_shmem:
            bng.open_shmem(self.annotation_handle, size)

    def disconnect(self, bng, vehicle):
        """
        This method is called when the vehicle is disconnected from the
        simulation. It's used to tell the simulation to close the shared memory
        used to exchange sensor data for this camera.

        Args:
            bng (:class:`.BeamNGpy`): Running instance of BeamNGpy.
            vehicle (:class:`.Vehicle`): The vehicle being disconnected.
        """
        if self.colour_shmem:
            bng.close_shmem(self.colour_handle)

        if self.depth_shmem:
            bng.close_shmem(self.depth_handle)

        if self.annotation_shmem:
            bng.close_shmem(self.annotation_handle)

    def encode_engine_request(self):
        """
        This method encodes a render request to the simulation engine along
        with the properties this camera is configured with.

        Returns:
            The request to the engine as a dictionary. This dictionary contains
            fields for each property of the requested render and which modes
            (colour, depth, annotation) to render in.
        """
        req = dict(type='Camera')

        if self.colour_shmem:
            req['color'] = self.colour_handle

        if self.depth_shmem:
            req['depth'] = self.depth_handle

        if self.annotation_shmem:
            req['annotation'] = self.annotation_handle

        req['pos'] = self.pos
        req['direction'] = self.direction
        req['fov'] = self.fov
        req['resolution'] = self.resolution
        req['near_far'] = self.near_far

        return req

    def decode_response(self, resp):
        """
        This method obtains sensor data written to shared memory and decodes
        them as images. The resulting data is returned as a dictionary. This
        dictionary contains an entry for each requested image type that is
        mapped to a :class:`PIL.Image` instance.

        Args:
            resp (dict): The raw sensor data as a dictionary that was returned
                         by the simulation.

        Returns:
            The decoded response as a dictionary.
        """
        decoded = dict(type='Camera')
        img_w = resp['width']
        img_h = resp['height']

        size = img_w * img_h * 4

        if self.colour_shmem:
            if 'color' in resp.keys():
                self.colour_shmem.seek(0)
                colour_d = self.colour_shmem.read(size)
                colour_d = np.frombuffer(colour_d, dtype=np.uint8)
                colour_d = colour_d.reshape(img_h, img_w, 4)
                decoded['colour'] = Image.fromarray(colour_d)
            else:
                print('Color buffer failed to render. Check that you '
                      'aren\'t running on low settings.', file=sys.stderr)

        if self.annotation_shmem:
            if 'annotation' in resp.keys():
                self.annotation_shmem.seek(0)
                annotate_d = self.annotation_shmem.read(size)
                annotate_d = np.frombuffer(annotate_d, dtype=np.uint8)
                annotate_d = annotate_d.reshape(img_h, img_w, 4)
                decoded['annotation'] = Image.fromarray(annotate_d)
            else:
                print('Annotation buffer failed to render. Check that you '
                      'aren\'t running on low settings.', file=sys.stderr)

        if self.depth_shmem:
            if 'depth' in resp.keys():
                self.depth_shmem.seek(0)
                depth_d = self.depth_shmem.read(size)
                depth_d = np.frombuffer(depth_d, dtype=np.float32)
                depth_d = depth_d / FAR
                depth_d = depth_d.reshape(img_h, img_w)
                depth_d = np.uint8(depth_d * 255)
                decoded['depth'] = Image.fromarray(depth_d)
            else:
                print('Depth buffer failed to render. Check that you '
                      'aren\'t running on low settings.', file=sys.stderr)

        return decoded

    def get_engine_flags(self):
        """
        Called to retrieve settings for the simulation engine. Depending on the
        types of data this camera is supposed to provide, this method returns
        a dictionary enabling certain render modes in the engine.
        """
        flags = dict()
        if self.annotation_shmem:
            flags['annotations'] = True
        return flags


class Lidar(Sensor):
    max_points = LIDAR_POINTS

    """
    The Lidar sensor provides 3D point clouds representing the environment
    as detected by a pulsing laser emitted from the vehicle. The range,
    position, and refresh rate of this sensor can be customised.
    """

    shmem_size = LIDAR_POINTS * 3 * 4

    def __init__(self, offset=(0, 0, 1.7), direction=(0, -1, 0), vres=32,
                 vangle=26.9, rps=2200000, hz=20, angle=360, max_dist=200,
                 visualized=True):
        self.handle = None
        self.shmem = None

        self.offset = offset
        self.direction = direction

        self.vres = vres
        self.vangle = vangle
        self.rps = rps
        self.hz = hz
        self.angle = angle
        self.max_dist = max_dist

        self.visualized = visualized

    def attach(self, vehicle, name):
        """
        Called when the lidar sensor is attached to a vehicle. This method
        allocates shared memory space to exchange lidar data with the engine.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle the sensor is being
                                         attached to.
            name (str): The name of the sensor.
        """
        pid = os.getpid()
        self.handle = '{}.{}.{}.lidar'.format(pid, vehicle.vid, name)
        self.shmem = mmap.mmap(0, Lidar.shmem_size, self.handle)
        log.debug('Bound memory for lidar: %s', self.handle)

    def detach(self, vehicle, name):
        """
        Called when the lidar sensor is detached from a vehicle. This method
        de-allocates the shared memory used to exchange lidar data with the
        engine.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle the sensor is being
                                         detached from.
            name (str): The name of the sensor.
        """
        self.shmem.close()

    def connect(self, bng, vehicle):
        bng.open_lidar(self.handle, vehicle, self.handle, Lidar.shmem_size,
                       offset=self.offset, direction=self.direction,
                       vres=self.vres, vangle=self.vangle, rps=self.rps,
                       hz=self.hz, angle=self.angle, max_dist=self.max_dist,
                       visualized=self.visualized)

    def disconnect(self, bng, vehicle):
        bng.close_lidar(self.handle)

    def encode_engine_request(self):
        """
        Called to obtain the engine request for this lidar sensor. Encodes the
        properties of this lidar to obtain data according to them.

        Returns:
            The engine request containing the settings of this lidar sensor as
            a dictionary.
        """
        req = dict(type='Lidar')
        req['name'] = self.handle
        return req

    def decode_response(self, resp):
        """
        Reads the raw point cloud the simulation wrote to the shared memory and
        creates a numpy array of points from them. The recoded response is
        returned as a dictionary with the numpy array in the ``points`` entry.

        Returns:
            The decoded response as a dictionary with the point cloud as a
            numpy array in the ``points`` entry. The numpy array is a linear
            sequence of coordinate triplets in the form of [x0, y0, z0, x1,
            y1, z1, ..., xn, yn, zn].
        """
        size = resp['size']
        self.shmem.seek(0)
        points_buf = self.shmem.read(size)
        points_buf = np.frombuffer(points_buf, dtype=np.float32)
        assert points_buf.size % 3 == 0
        resp = dict(type='Lidar')
        resp['points'] = points_buf
        return resp

    def get_engine_flags(self):
        """
        Returns: a dictionary with the engine flag to enable Lidar.
        """
        flags = dict(lidar=True)
        return flags


class GForces(Sensor):
    """
    This sensor is used to obtain the GForces acting on a vehicle.

    # TODO: GForce sensor for specific points on/in the vehicle
    """

    def encode_vehicle_request(self):
        req = dict(type='GForces')
        return req


class Electrics(Sensor):
    """
    This sensor is used to retrieve various values made available by the car's
    eletrics systems. These values include:

    # TODO: List all the electrics.lua values.
    """

    def encode_vehicle_request(self):
        req = dict(type='Electrics')
        return req


class Damage(Sensor):
    """
    The damage sensor retrieves information about how damaged the structure
    of the vehicle is. It's important to realise that this is a sensor that has
    no analogue in real life as it returns a perfect knowledge overview of how
    deformed the vehicle is. It's therefore more of a ground truth than
    simulated sensor data.
    """

    def encode_vehicle_request(self):
        req = dict(type='Damage')
        return req


class Timer(Sensor):
    """
    The timer sensor keeps track of the time that has passed since the
    simulation started. It provides that information in seconds relative to the
    scenario start and does not represent something like a day time or date. It
    properly handles pausing the simulation, meaning the value of the timer
    sensor does not progress while the simulation is paused.

    When polled, this sensor provides the time in seconds since the start of
    the scenario in a dictionary under the 'time' key.
    """

    def encode_engine_request(self):
        req = dict(type='Timer')
        return req
