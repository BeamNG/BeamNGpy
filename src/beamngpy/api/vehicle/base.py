from __future__ import annotations

from typing import TYPE_CHECKING

from beamngpy.logging import _generate_docstring

if TYPE_CHECKING:
    from beamngpy.vehicle import Vehicle


class VehicleApi:
    """
    An API class for in-game logging of vehicle data.

    Args:
        vehicle: An instance of a vehicle object.
    """

    def __init__(self, vehicle: Vehicle):
        self._vehicle = vehicle
        self._send = vehicle._send
        self._logger = vehicle.logger
        self.__doc__ = _generate_docstring(self)
