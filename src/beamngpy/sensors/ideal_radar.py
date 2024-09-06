from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID, BNGError
from beamngpy.types import StrDict

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

__all__ = ["IdealRadar"]


class IdealRadar(CommBase):
    """
    This automated sensor provides the user with data relating to vehicles within a close proximity to its position.  Quantities such as velocity and acceleration
    are available for these vehicles, in a reference frame local the sensor.  These sensors can be attached to any vehicle, or to any fixed point on the map.

    Args:
        name: A unique name for this ideal RADAR sensor.
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this sensor should be attached. Note: a vehicle must be provided for the ideal RADAR sensor.
        gfx_update_time: The gfx-step time which should pass between sensor reading updates to the user, in seconds.
        physics_update_time: The physics-step time which should pass between actual sampling the sensor, in seconds.
        is_send_immediately: A flag which indicates if the readings should be sent back as soon as available or upon graphics step updates, as bulk.
    """

    def __init__(
        self,
        name: str,
        bng: BeamNGpy,
        vehicle: Vehicle,
        gfx_update_time: float = 0.0,
        physics_update_time: float = 0.01,
        is_send_immediately: bool = False,
    ):
        super().__init__(bng, vehicle)

        self.logger = getLogger(f"{LOGGER_ID}.IdealRADAR")
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.name = name
        self.vehicle = vehicle
        self.is_send_immediately = is_send_immediately

        # Create and initialise this sensor in the simulation.
        self._open_ideal_radar(
            name, vehicle, gfx_update_time, physics_update_time, is_send_immediately
        )

        # Fetch the unique Id number (in the simulator) for this ideal RADAR sensor.
        self.sensor_id = self._get_id()
        self.logger.debug("idealRADAR - sensor created: " f"{self.name}")

    def remove(self) -> None:
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_ideal_radar()
        self.logger.debug("idealRADAR - sensor removed: " f"{self.name}")

    def poll(self) -> StrDict:
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary containing the sensor readings data.
            The ideal RADAR sensor detects the closest vehicles within a hard-coded distance from the ego vehicle, which are in front of the ego vehicle and at the same direction.
            The number of closest vehicles has been hard-coded as 4; one may edit this from the Lua end.
            For each of these vehicles the sensor returns:
            vehicleID: the vehicle's unique id.
            width: the vehicle's width.
            length: the vehicle's length.
            distToPlayerVehicleSq: the squared distance of the ego vehicle and this vehicle.
            relDistX, relDistY: The relative distance to the ego vehicle front position, longitudinal and lateral.
            relVelX, relVelY: The relative velocity wrt the ego vehicle frame, longitudinal and lateral.
            relAccX, relAccY: The relative acceleration wrt the ego vehicle frame, longitudinal and lateral.
            vel: velocity vector of this vehicle.
            acc: acceleration vector of this vehicle.
        """

        # Send and receive a request for readings data from this sensor.
        readings_data = []
        if self.is_send_immediately:
            # Get the most-recent single reading from vlua.
            readings_data = self._poll_ideal_radar_VE()
        else:
            # Get the bulk data from ge lua.
            readings_data = self._poll_ideal_radar_GE()

        self.logger.debug(
            "Powertrain - sensor readings received from simulation: " f"{self.name}"
        )
        return readings_data

    def send_ad_hoc_poll_request(self) -> int:
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            A unique Id number for the ad-hoc request.
        """
        self.logger.debug("idealRADAR - ad-hoc polling request sent: " f"{self.name}")
        return int(
            self.send_recv_ge(
                "SendAdHocRequestIdealRADAR", name=self.name, vid=self.vehicle.vid
            )["data"]
        )

    def is_ad_hoc_poll_request_ready(self, request_id: int) -> bool:
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug(
            "idealRADAR - ad-hoc polling request checked for completion: "
            f"{self.name}"
        )
        return self.send_recv_ge(
            "IsAdHocPollRequestReadyIdealRADAR", requestId=request_id
        )["data"]

    def collect_ad_hoc_poll_request(self, request_id: int) -> StrDict:
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            The readings data.
        """
        readings = self.send_recv_ge(
            "CollectAdHocPollRequestIdealRADAR", requestId=request_id
        )["data"]
        self.logger.debug(
            "idealRADAR - ad-hoc polling request returned and processed: "
            f"{self.name}"
        )
        return readings

    def set_requested_update_time(self, requested_update_time: float) -> None:
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time: The new requested update time.
        """
        self.send_ack_ge(
            "SetIdealRADARRequestedUpdateTime",
            ack="CompletedSetIdealRADARRequestedUpdateTime",
            name=self.name,
            vid=self.vehicle.vid,
            GFXUpdateTime=requested_update_time,
        )

    def _get_id(self) -> int:
        return int(self.send_recv_ge("GetIdealRADARId", name=self.name)["data"])

    def _open_ideal_radar(
        self,
        name: str,
        vehicle: Vehicle,
        gfx_update_time: float,
        physics_update_time: float,
        is_send_immediately: bool,
    ) -> None:
        data: StrDict = dict()
        data["name"] = name
        data["vid"] = vehicle.vid
        data["GFXUpdateTime"] = gfx_update_time
        data["physicsUpdateTime"] = physics_update_time
        data["isSendImmediately"] = is_send_immediately
        self.send_ack_ge(type="OpenIdealRADAR", ack="OpenedIdealRADAR", **data)
        self.logger.info(f'Opened idealRADAR sensor: "{name}"')

    def _close_ideal_radar(self) -> None:
        self.send_ack_ge(
            type="CloseIdealRADAR",
            ack="ClosedIdealRADAR",
            name=self.name,
            vid=self.vehicle.vid,
        )
        self.logger.info(f'Closed idealRADAR sensor: "{self.name}"')

    def _poll_ideal_radar_GE(self) -> StrDict:
        return self.send_recv_ge("PollIdealRADARGE", name=self.name)["data"]

    def _poll_ideal_radar_VE(self) -> StrDict:
        return self.send_recv_veh(
            "PollIdealRADARVE", name=self.name, sensorId=self.sensor_id
        )["data"]
