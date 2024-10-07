from __future__ import annotations

from .base import VehicleApi


class AccApi(VehicleApi):
    """
    A base API class from which all the API communicating with a vehicle derive.

    Args:
        vehicle: An instance of a vehicle object.
    """

    def start(self, sp: float, inputFlag: bool) -> None:
        """
        Starts ACC

        Args:
            vehicle: An instance of a vehicle object.
        """
        data = dict(
            type="LoadACC"
        )  # dict(type='LoadACC', speed=sp, debugFlag=inputFlag)
        data["speed"] = sp
        data["debugFlag"] = inputFlag
        self._send(data).ack("ACCloaded")
        log_msg = "Started ACC."
        self._logger.info(log_msg)

    def stop(self) -> None:
        """
        Stops ACC.
        """
        data = dict(type="UnloadACC")
        self._send(data).ack("ACCunloaded")
        self._logger.info("Stopped ACC.")
