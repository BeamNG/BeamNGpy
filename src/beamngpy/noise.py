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
import numpy as np
from PIL import Image
from abc import abstractmethod

class Noise(AbstractSensor):
    
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
        # if noise doesn't have it, maybe the sensor has it
        return getattr(self._sensor, name) 
    
    def __setattr__(self, name, value):
        #it won't be possible to set values in the sensor class this way
        if name == "data":
            super().__setattr__(name, value)
        else:
            self.__dict__[name] = value

class WhiteGaussianRGBNoise(Noise):

    def __init__(self, sensor, mu, sigma):
        super().__init__(sensor)
        self._sigma = sigma
        self._mu = mu
    
    def _generate_noisy_data(self):
        image = np.array(self._sensor.data["colour"], dtype=np.float64)/255
        rgb_noise = np.random.normal(self._mu, self._sigma, image.shape)
        assert(not(np.array_equal(image, image+rgb_noise)))
        image = image+rgb_noise
        image[image>1] = 1
        image[image<0] = 0
        image = Image.fromarray((image*255).astype(np.uint8))
        self._data = {'colour':image}

