from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle


class AdasUltrasonicApi(CommBase):
    """
    An API for ultrasonic sensor-based parking assistance and blind spot detection of BeamNG.tech vehicle.
    A configuration with 10 ultrasonic sensors is used for parking assistance, 4 of which can also be used for blind spot detection.
    The parking assistance activates only at speeds below 12.6 km/h. It makes sure to slow down and stop the vehicle to avoid a collision.
    The blind spot detection is visualised through HUD notifications.

    Usage with a steering wheel controller and pedals is recommended.

    Args:
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this API should be attached.
    """

    def __init__(self, bng: BeamNGpy, vehicle: Vehicle):
        super().__init__(bng, vehicle)

        self.logger = getLogger(f"{LOGGER_ID}.AdasUltrasonicApi")
        self.logger.setLevel(DEBUG)

        self.running = False

    def start(
        self,
        parkAssist: bool = True,
        blindSpot: bool = True,
        crawl: bool = True,
        is_visualised: bool = True
    ) -> None:
        """
        Starts Ultrasonic ADAS features. Preferrably do this when the vehicle is still.

        Args:
            parkAssist: whether to enable parking assistance.
            blindSpot: whether to enable blind spot detection.
            crawl: whether the vehicle's automatic transmission moves without throttle when put in gear.
            is_visualised: whether the ultrasonic sensors should be is_visualised.
        """
        if self.running:
            self.logger.warning("Ultrasonic ADAS is already running.")
            return

        self.send_ack_ge(
            "LoadUltrasonicADAS",
            ack="UltrasonicADASloaded",
            vid=self.vehicle.vid,
            parkAssist=parkAssist,
            blindSpot=blindSpot,
            crawl=crawl,
            is_visualised=is_visualised,
        )

        self.running = True

        self.logger.info("Started Ultrasonic ADAS.")

    def stop(self) -> None:
        """
        This stops Ultrasonic ADAS features from the associated vehicle.
        """
        if not self.running:
            self.logger.warning("Ultrasonic ADAS is not running.")
            return

        self.send_ack_ge(
            "UnloadUltrasonicADAS",
            ack="UltrasonicADASunloaded",
        )

        self.running = False

        self.logger.info("Stopped Ultrasonic ADAS.")
