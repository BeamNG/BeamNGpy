"""
This module implements various sensors that can be attached to vehicles to
extract data from simulations.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from beamngpy.types import StrDict

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle


class Sensor(dict):
    """
    Sensor meta-class declaring methods common to them.
    """

    def __init__(self):
        self.data: StrDict = self  # backwards compatibility

    def replace(self, data: StrDict):
        self.clear()
        self.update(data)

    def attach(self, vehicle: Vehicle, name: str) -> None:
        """
        Called when the sensor is attached to a :class:`.Vehicle` instance.
        Used to perform sensor setup code before the simulation is started.
        This is called *after* the sensor has been entered into the vehicle's
        map of sensors under the given name.

        Args:
            vehicle: The vehicle instance the sensor is being attached to.
            name: The name the sensor is known under to the vehicle.
        """
        pass

    def detach(self, vehicle: Vehicle, name: str) -> None:
        """
        Called when the sensor is detached from a :class:`.Vehicle` instance.
        Used to perform sensor teardown code after the simulation is finished.
        This is called *after* the sensor has been removed from the
        vehicle's map of sensors under the given name.

        Args:
            vehicle: The vehicle instance the sensor is being detached from.
            name: The name the sensor was known under to the vehicle.
        """
        pass

    def encode_engine_request(self) -> StrDict | None:
        """
        Called to retrieve this sensor's data request to the engine as a
        dictionary. The dictionary returned by this method will be bundled
        along the vehicle's other sensors' requests as a SensorRequest to the
        simulator's engine.

        Note:
            Sensors require corresponding code in the simulator to handle
            requests.

        Returns:
            The request to send to the engine as a dictionary.
        """
        return None

    def encode_vehicle_request(self) -> StrDict:
        """
        Called to retrieve this sensor's request to the vehicle as a
        dictionary. The dictionary returned by this method will be bundled
        along the vehicle's other sensors' requests as a SensorRequest to the
        attached vehicle.

        Note:
            Sensors require corresponding code in the simulator to handle
            requests.

        Returns:
            The request to send to the vehicle as a dictionary.
        """
        return {}

    def decode_response(self, resp: StrDict) -> StrDict:
        """
        Called to do post-processing on sensor data obtained from the
        simulation. This method is called after raw simulation data is received
        and the resulting processed data is considered the result of a sensor
        request.
        """
        return resp

    def connect(self, bng: BeamNGpy, vehicle: Vehicle) -> None:
        """
        Called when the attached vehicle is being initialised in the
        simulation. This method is used to perform setup code that requires the
        simulation to be running.
        """
        pass

    def disconnect(self, bng: BeamNGpy, vehicle: Vehicle) -> None:
        """
        Called when the attached vehicle is being removed from simulation. This
        method is used to perform teardown code after the simulation.
        """
        pass
