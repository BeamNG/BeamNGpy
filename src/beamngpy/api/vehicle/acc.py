from __future__ import annotations

from .base import VehicleApi


class AccApi(VehicleApi):
    """
    An API for Adaptive Cruise Control (experimental feature) of BeamNG.tech vehicle.

    Args:
        vehicle: An instance of a vehicle object.
    """

    def start(self, speed: float, debug: bool = False) -> None:
        """
        Load and start ACC on this vehicle.

        Args:
            speed: Target cruise speed in m/s.
            debug: When True, enable ACC debug logging and CSV output.
        """
        data = dict(type="LoadACC", speed=speed, debug=debug)
        self._send(data).ack("ACCloaded")
        self._logger.info("Started ACC.")

    def stop(self) -> None:
        """
        Unload ACC from this vehicle.
        """
        data = dict(type="UnloadACC")
        self._send(data).ack("ACCunloaded")
        self._logger.info("Stopped ACC.")

    def change_speed(self, speed: float) -> None:
        """
        Change the ACC target speed while ACC is loaded.

        Args:
            speed: New target speed in m/s.
        """
        data = dict(type="ChangeACCSpeed", speed=speed)
        self._send(data).ack("ACCSpeedChanged")
        self._logger.info(f"Changed ACC target speed to {speed} m/s.")
