from __future__ import annotations

from .base import VehicleApi


class AccApi(VehicleApi):
    """
    An API for Adaptive Cruise Control (experimental feature) of BeamNG.tech vehicle.

    Args:
        vehicle: An instance of a vehicle object.
        speed: the target speed of the vehicle, when it doeasn't follow an ACC-eligible preceding vehicle.
        flag: default set to True
    """

    def start(self, vehid: str, sp: float, inputFlag: bool) -> None:
        """
        Starts ACC

        Args:
            vehicle: An instance of a vehicle object.
            speed: the target speed of the vehicle, when it doeasn't follow an ACC-eligible preceding vehicle.
            input flag: used for debugging purpose
        """
        data = dict(
            type="LoadACC"
        )
        data["vid"] = vehid  # The vehicle’s ID
        data["speed"] = sp
        data["debugFlag"] = inputFlag
        self._send(data).ack("ACCloaded")
        log_msg = "Started ACC."
        self._logger.info(log_msg)

    def stop(self) -> None:
        """
        This stops ACC function from the associated vehicle.
        """
        data = dict(type="UnloadACC")
        self._send(data).ack("ACCunloaded")
        self._logger.info("Stopped ACC.")
