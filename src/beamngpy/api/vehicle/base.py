from __future__ import annotations

from typing import TYPE_CHECKING

from beamngpy.logging import generate_docstring

if TYPE_CHECKING:
    from beamngpy.vehicle import Vehicle


class VehicleApi:
    def __init__(self, vehicle: Vehicle):
        self._vehicle = vehicle
        self._send = vehicle._send
        self._logger = vehicle.logger
        self.__doc__ = generate_docstring(self)
