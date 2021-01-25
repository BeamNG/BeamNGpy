"""
.. module:: sensors
    :platform: Windows
    :synopsis: Module containing a wrapper class for the sensors, for
               noise generation.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>

This module implements a wrapper class for the sensors, so that it is possible
to generate non-ideal data.
"""

from .sensors import AbstractSensor
import numpy as np
from PIL import Image
from abc import abstractmethod
from skimage.util import noise as skinoise
from skimage.util import dtype as skitype


class Noise(AbstractSensor):
    """
    Noise meta-class managing the data attribute and defining a common
    interface. The Noise class wraps any sensor and instances of the noise
    subclass can (almost) be used as if it was a sensor on its own. Multiple
    noise subclasses can be chained. If a member function is not part of Noise,
    the call will be forwarded to the Sensor it contains. Outside the .data
    property it is not possible to set attributes of the sensor class.
    """

    def __init__(self, sensor):
        self._sensor = sensor
        self._data = dict()
        super().__init__()

    @abstractmethod
    def _generate_noisy_data(self):
        """
        Generate and cache noisy data in self._data
        """
        pass

    @property
    def data(self):
        """
        Returns the added up cached noise with the one from sensor.data.
        This does not affect the data from the original sensor.
        """
        data = self._sensor.data.copy()
        data.update(self._data)
        return data

    @data.setter
    def data(self, data):
        """
        Setting sensor data and generating noise.
        """
        self._sensor.data = data
        self._generate_noisy_data()

    @data.deleter
    def data(self):
        self._data = None

    def __getattr__(self, name):
        """
        If __getattribute__ fails on this class, then we are forwarding the
        request to the contained sensor.
        """
        return getattr(self._sensor, name)

    def __setattr__(self, name, value):
        """
        Intercepts any setter request, only setter requests for this class are
        handled, making it impossible to directly set attributes of the
        contained _sensor object.
        """
        if name == "data":
            super().__setattr__(name, value)
        else:
            self.__dict__[name] = value


class RandomImageNoise(Noise):
    """
    A noise generating class for the Camera Sensor.
    """

    def __init__(self, sensor, colour=True, depth=True, **kwargs):
        """
        A wrapper class for the random_noise function
        from the skimage.util.noise package.
        The default noise type is gaussian additive noise.
        For a list of arguments refer to the
        scikit-image documentation.
        The 'seed' argument is disabled, since it would cause the
        class to generate the same noise for all images.

        Args:
            colour(bool): Whether to apply noise to the colour image.
            depth(bool): Whether to apply noise to the depth image.
        """
        super().__init__(sensor)
        img_types = ['colour', 'depth']
        use = [colour, depth]
        self._data = dict()
        self._img_types = [t for t, do in zip(img_types, use)]
        kwargs.update({'seed': None})
        self._generate_noise = lambda img: skinoise.random_noise(img,
                                                                     **kwargs)

    def _generate_noisy_data(self):
        """
        This member function is called to generate the noise
        and apply it to the colour and/or depth image from the camera sensor.
        """
        for img_type in self._img_types:
            img = np.array(self._sensor.data[img_type])
            img = self._generate_noise(img)
            img = skitype.img_as_ubyte(img)
            img = Image.fromarray(img)
            self._data.update({img_type: img})


class RandomLIDARNoise(Noise):
    """
    Class generating gaussian additive noise for the LIDAR point cloud.
    """

    def __init__(self, sensor, mean=0.0, var=.01):
        """
        A wrapper class that applies gaussian additive noise to LIDAR
        point clouds.
        Before applying the noise, the point cloud array is scaled
        to be in [-1, +1], choose mean and variance accordingly.

        Args:
            mean(float): mean of gaussian distribution
            var(float): variance of gaussian distribution
        """
        super().__init__(sensor)
        self._data = dict()

        self._mean = mean
        self._var = var

    def _generate_noisy_data(self):
        """
        This member function is called to generate the noise
        and applies it to the point cloud.
        """
        point_cloud = np.array(self._sensor.data['points'])
        if point_cloud.size > 0:
            point_cloud = point_cloud.astype(np.float64)
            magnitude = max(point_cloud.max(), np.abs(point_cloud.min()))
            point_cloud /= magnitude
            point_cloud = skinoise.random_noise(point_cloud,
                                                mode='gaussian',
                                                mean=self._mean,
                                                var=self._var)
            point_cloud = point_cloud*magnitude
            point_cloud = skitype.img_as_float32(point_cloud)
        else:
            point_cloud = self._sensor.data['points']
        self._data.update({'points': point_cloud})
