"""
.. module:: sensors
    :platform: Windows
    :synopsis: Module containing the various sensors one can attach to a
               vehicle.
    :noindex:

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
.. moduleauthor:: Sedonas <https://github.com/Sedonas>
.. moduleauthor:: Dave Stark <dstark@beamng.gmbh>

This module implements various sensors that can be attached to vehicles to
extract data from simulations.
"""
from abc import ABC, abstractmethod


class AbstractSensor(ABC):
    """
    Abstract Sensor class declaring properties common to the ordinary and noise
    sensors.
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

    def __getitem__(self, item):
        return self.data[item]

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
