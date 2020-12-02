"""
.. module:: sensors
    :platform: Windows
    :synopsis: Module containing a wrapper class for the sensors, for noise generation.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>

This module implements a wrapper class for the sensors, so that it is possible to generate non-ideal data.
"""

from .sensors import AbstractSensor
import numpy as np
from PIL import Image
from abc import abstractmethod

class Noise(AbstractSensor):
    """
    Noise meta-class managing the data attribute and defining a common interface.
    The Noise class wraps any sensor and instances of the noise subclass 
    can (almost) be used as if it was a sensor on its own.
    Multiple noise subclasses can be chained.
    If a member function is not part of Noise, the call will be forwarded to the Sensor it contains.
    Outside the .data property it is not possible to set attributes of the sensor class.  
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
        """
        data = self._sensor.data
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
        If __getattribute__ fails on this class, then we are forwarding the request to the contained sensor.
        """
        return getattr(self._sensor, name) 
    
    def __setattr__(self, name, value):
        """
        Intercepts any setter request, only setter requests for this class are handled,
        making it impossible to directly set attributes of the contained _sensor object.
        """
        if name == "data":
            super().__setattr__(name, value)
        else:
            self.__dict__[name] = value

class WhiteGaussianRGBNoise(Noise):
    """
    A noise generating class for the Camera Sensor. 
    It generates white gaussian noise and applies it to the image.
    """

    def __init__(self, sensor, mu, sigma):
        """
        Setting parameters for the Gauss distribution.

        Args:
            mu (float): mean (centre) of the function
            sigma (float): standard deviation (width) of the function
        """
        super().__init__(sensor)
        self._sigma = sigma
        self._mu = mu
    
    def _generate_noisy_data(self):
        """
        This member function is called to generate the noise 
        and apply it to the RGB image in the data dictionary.
        """
        image = np.array(self._sensor.data["colour"], dtype=np.float64)/255
        rgb_noise = np.random.normal(self._mu, self._sigma, image.shape)
        image = image+rgb_noise
        image[image>1] = 1
        image[image<0] = 0
        image = Image.fromarray((image*255).astype(np.uint8))
        self._data = {'colour':image}

