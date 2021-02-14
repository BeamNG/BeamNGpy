"""
.. module:: sensors
    :platform: Windows
    :synopsis: Module containing the various sensors one can attach to a
               vehicle.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
.. moduleauthor:: Sedonas <https://github.com/Sedonas>

This module implements various sensors that can be attached to vehicles to
extract data from simulations.
"""
import base64
import logging as log
import mmap
import os
import sys
from abc import ABC, abstractmethod

import numpy as np
from PIL import Image

NEAR = 0.01
FAR = 1000

LIDAR_POINTS = 2000000

class AbstractSensor(ABC):
    """
    Abstract Sensor class declaring properties common to the ordinary and noise sensors.
    """
    
    @property
    @abstractmethod
    def data(self):
        pass   

    @data.setter
    @abstractmethod
    def data(self, data):
        pass

    @data.deleter
    @abstractmethod
    def data(self):
        pass
    
class Sensor(AbstractSensor):
    """
    Sensor meta-class declaring methods common to them.
    """
    def __init__(self):
        self._data = dict()     

    @property
    def data(self):
        """
        Property used to store sensor readings.
        """
        return self._data

    @data.setter
    def data(self, data):
        self._data = data
    
    @data.deleter
    def data(self):
        self._data = None

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
        del self.data

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
                 colour=False, depth=False, depth_distance=(NEAR, FAR),
                 annotation=False):
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
            depth_distance (tuple): (near,far) tuple of the distance range
                                    depth values should be mapped between.
                                    For example, a distance_scale of (10, 50)
                                    would mean geometry closer than 10 would
                                    be mapped to black and geometry further
                                    than 50 would be mapped to white. All
                                    distances in-between are interpolated
                                    accordingly.
            annotation (bool): Whether to output annotation information.
        """
        super().__init__()
        self.pos = pos
        self.direction = direction
        self.fov = fov
        self.resolution = resolution
        self.near_far = near_far

        self.colour = colour
        self.depth = depth
        self.depth_distance = depth_distance
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

        req['pos'] = [float(f) for f in self.pos]
        req['direction'] = [float(f) for f in self.direction]
        req['fov'] = self.fov
        req['resolution'] = [int(i) for i in self.resolution]
        req['near_far'] = [float(f) for f in self.near_far]

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

                # Use linear interpolation to map the depth values
                # between lightness values 0-255. Any distances outside
                # of the scale are clamped to either 0 or 255
                # respectively.
                depth_d = np.interp(depth_d, [self.depth_distance[0],
                            self.depth_distance[1]], [255, 0], left=255,
                            right=0)

                depth_d = depth_d.reshape(img_h, img_w)
                depth_d = np.uint8(depth_d)
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
        super().__init__()
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
    def __init__(self):
        super().__init__()

    def encode_vehicle_request(self):
        req = dict(type='GForces')
        return req


class Electrics(Sensor):
    """
    This sensor is used to retrieve various values made available by the car's
    eletrics systems. These values include:

    # TODO: List all the electrics.lua values.
    - abs (int): ABS state
    - abs_active (bool):
    - airspeed (float): Airspeed
    - airflowspeed (float):
    - altitude (float): Z axis position
    - avg_wheel_av (float):
    - brake (int): Brake value
    - brake_lights (int):
    - brake_input (int): Brake input value
    - check_engine (bool): Check engine light state.
    - clutch (int): Clutch value
    - clutch_input (int): Clutch input value
    - clutch_ratio (int):
    - driveshaft (float): Driveshaft
    - engine_load (float):
    - engine_throttle (int): Engine throttle state
    - esc (int): ESC state. 0 = not present/inactive, 1 = disabled,
                 Blink = active
    - esc_active (bool):
    - exhaust_flow (float):
    - fog_lights (int): Fog light state
    - fuel (float): Percentage of fuel remaining.
    - fuel_capacity (int): Total Fuel Capacity [L].
    - fuel_volume (float):
    - gear (int):
    - gear_a (int): Gear selected in automatic mode.
    - gear_index (int):
    - gear_m (int): Gear selected in manual mode.
    - hazard (int): Hazard light state
    - hazard_signal (bool):
    - headlights (int):
    - highbeam (int): High beam state
    - horn (int):
    - ignition (bool): Engine state
    - left_signal (bool):
    - lightbar (int): Lightbar state
    - lights (int): General light state. 1 = low, 2 = high
    - lowbeam (int): Low beam state
    - lowfuel (bool): Low fuel indicator
    - lowhighbeam (int): Low-high beam state
    - lowpressure (int): Low fuel pressure indicator
    - oil (int):
    - oil_temperature (float): Oil temperature [C].
    - parking (int): Parking lights on/off (not implemented yet)
    - parkingbrake (float): Parking brake state. 0.5 = halfway on
    - parkingbrake_input (int): Parking brake input state
    - radiator_fan_spin (int):
    - reverse (int): Reverse gear state
    - right_signal (bool):
    - rpm (float): Engine RPM
    - rpmspin (float):
    - rpm_tacho (float):
    - running (bool): Engine running state
    - signal_l (int): Left signal state. 0.5 = halfway to full blink
    - signal_r (int): Right signal state. 0.5 = halfway to full blink
    - steering (int): Steering state
    - steering_input (int): Steering input state
    - tcs (int): TCS state. 0 = not present/inactive, 1 = disabled,
                 Blink = active
    - tcs_active (bool):
    - throttle (int): Throttle state
    - throttle_factor (int):
    - throttle_input (int): Throttle input state
    - turnsignal (int): Turn signal value. -1 = Left, 1 = Right,
                        gradually 'fades' between values. Use "signal_L" and
                        "signal_R" for flashing indicators.
    - two_step (bool):
    - water_temperature (float): Water temperature [C].
    - wheelspeed (float): Wheel speed [m/s].
    """
    name_map = {
        'absActive': 'abs_active',
        'avgWheelAV': 'avg_wheel_av',
        'brakelights': 'brake_lights',
        'checkengine': 'check_engine',
        'clutchRatio': 'clutch_ratio',
        'engineLoad': 'engine_load',
        'engineThrottle': 'engine_throttle',
        'escActive': 'esc_active',
        'exhaustFlow': 'exhaust_flow',
        'fog': 'fog_lights',
        'fuelVolume': 'fuel_volume',
        'fuelCapacity': 'fuel_capacity',
        'gear_A': 'gear_a',
        'gearIndex': 'gear_index',
        'gear_M': 'gear_m',
        'hazard_enabled': 'hazard_signal',
        'isShifting': 'is_shifting',
        'lights_state': 'headlights',
        'oiltemp': 'oil_temperature',
        'radiatorFanSpin': 'radiator_fan_spin',
        'rpmTacho': 'rpm_tacho',
        'signal_L': 'signal_l',
        'signal_left_input': 'left_signal',
        'signal_R': 'signal_r',
        'signal_right_input': 'right_signal',
        'tcsActive': 'tcs_active',
        'throttleFactor': 'throttle_factor',
        'twoStep': 'two_step',
        'watertemp': 'water_temperature',
    }
    def __init__(self):
        super().__init__()

    def _rename_values(self, vals):
        """
        The values returned from the game often don't follow any naming
        convention and especially don't follow this library's, so we rename
        some of them here to be more consistent.
        """

        for k, v in Electrics.name_map.items():
            if k in vals:
                vals[v] = vals[k]
                del vals[k]
        return vals

    def _reassign_values(self, vals):
        if 'left_signal' in vals:
            vals['left_signal'] = vals['left_signal'] == 1
        if 'right_signal' in vals:
            vals['right_signal'] = vals['right_signal'] == 1
        if 'hazard_signal' in vals:
            vals['hazard_signal'] = vals['hazard_signal'] == 1
        return vals

    def encode_vehicle_request(self):
        req = dict(type='Electrics')
        return req

    def decode_response(self, resp):
        if 'values' in resp:
            ret = self._rename_values(resp['values'])
            ret = self._reassign_values(ret)
            return ret
        return None


class Damage(Sensor):
    """
    The damage sensor retrieves information about how damaged the structure
    of the vehicle is. It's important to realise that this is a sensor that has
    no analogue in real life as it returns a perfect knowledge overview of how
    deformed the vehicle is. It's therefore more of a ground truth than
    simulated sensor data.
    """
    def __init__(self):
        super().__init__()

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
    def __init__(self):
        super().__init__()

    def encode_engine_request(self):
        req = dict(type='Timer')
        return req

class State(Sensor):
    """
    The state sensor monitors general stats of the vehicle, such as position, direction, velocity, etc.
    It is a default sensor every vehicle has and is used to update the vehicle.state attribute.
    """
    def __init__(self):
        super().__init__()

    def encode_vehicle_request(self):
        req  = dict(type='State')
        return req