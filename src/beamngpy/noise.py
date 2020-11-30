"""
.. module:: sensors
    :platform: Windows
    :synopsis: Module containing the various sensors one can attach to a
               vehicle.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>

This module implements various sensors that can be attached to vehicles to
extract data from simulations.
"""

from .sensors import AbstractSensor
from abc import ABC, abstractmethod
import numpy as np
from PIL import Image

class Noise(ABC, AbstractSensor):
    
    def __init__(self, sensor):
        self._sensor = sensor
        self._data = dict()
        super().__init__()
    
    @abstractmethod
    def _generate_noisy_data(self):
        """
        generate and cache noisy data in self._data 
        """
        pass

    @property
    def data(self):
        """
        Returns the added up cached noise with the one from sensor.data.
        """
        data = self._sensor.data
        data.update(self._data)
        return data
    
    @data.setter
    def data(self, data):
        """
        setting sensor data and generating noisy data
        """
        self._sensor.data = data
        self._generate_noisy_data()
    
    @data.deleter
    def data(self):
        self._data = None

    def __getattr__(self, name):
        return getattr(self._sensor, name)
    
    def __setattr__(self, name, value):
        if name in ("_sensor"):
            self.__dict__[name] = value
        else:
            setattr(self._sensor, name, value)

class WhiteGaussianRGBNoise(Noise):

    def __init__(self, sensor, mu, sigma):
        super().__init__(sensor)
        self._img_shape = sensor.resolution + (3,)
        self.sigma = sigma
    
    def _generate_noisy_data(self):
        image = np.asarray(self._sensor.data["colour"], dtype=np.float64)/255
        rgb_noise = np.random.normal(self.mu, self.sigma,self._img_shape)
        image = image+rgb_noise
        image[image>1] = 1
        image[image<0] = 0
        image = Image.fromarray(image*255)
        self._data = {'colour':image}