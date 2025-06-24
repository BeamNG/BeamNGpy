from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle


class AdasUssApi(CommBase):
    """
    An API for ultrasonic sensor-based parking assistance and blind spot detection of BeamNG.tech vehicle.

    Args:
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this API should be attached.
    """

    def __init__(self, bng: BeamNGpy, vehicle: Vehicle):
        super().__init__(bng, vehicle)

        self.logger = getLogger(f"{LOGGER_ID}.AdasUssApi")
        self.logger.setLevel(DEBUG)

        self.running = False

    def start(
        self,
        parkAssist: bool = True,
        blindSpot: bool = True,
        crawl: bool = True,
        visualised: bool = True
    ) -> None:
        """
        Starts USS ADAS features. Preferrably do this when the vehicle is still.

        Args:
            parkAssist: whether to enable parking assistance.
            blindSpot: whether to enable blind spot detection.
            crawl: whether the vehicle's transmission has inherent crawl.
            visualised: whether the ultrasonic sensors should be visualised.
        """
        if self.running:
            self.logger.warning("USS ADAS is already running.")
            return

        self.send_ack_ge(
            "LoadUSSADAS",
            ack="USSADASloaded",
            vid=self.vehicle.vid,
            parkAssist=parkAssist,
            blindSpot=blindSpot,
            crawl=crawl,
            visualised=visualised,
        )

        self.running = True

        self.logger.info("Started USS ADAS.")

    def stop(self) -> None:
        """
        This stops USS ADAS features from the associated vehicle.
        """
        if not self.running:
            self.logger.warning("USS ADAS is not running.")
            return

        self.send_ack_ge(
            "UnloadUSSADAS",
            ack="USSADASunloaded",
        )

        self.running = False

        self.logger.info("Stopped USS ADAS.")
