from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from beamngpy.vehicle import Vehicle


class VehicleApi:
    def __init__(self, vehicle: Vehicle):
        self.vehicle = vehicle
        self.send = vehicle.send
        self.logger = vehicle.logger
