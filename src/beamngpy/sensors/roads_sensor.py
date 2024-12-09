from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID
from beamngpy.types import StrDict

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

__all__ = ["RoadsSensor"]


class RoadsSensor(CommBase):
    """
    A sensor which gives geometric and semantic data of the road; this data is the parametric cubic equations for the left and right roadedge
    and the centerline, as well as 4 points of the centerline.

    Args:
        name: A unique name for this roads sensor.
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this sensor should be attached. Note: a vehicle must be provided for the roads sensor.
        gfx_update_time: The gfx-step time which should pass between sensor reading updates to the user, in seconds.
        physics_update_time: The physics-step time which should pass between actual sampling the sensor, in seconds.
        is_send_immediately: A flag which indicates if the readings should be sent back as soon as available or upon graphics step updates, as bulk.
        is_visualised: A flag which indicates if the sensor will visualize its debug outputs.
    """

    def __init__(
        self,
        name: str,
        bng: BeamNGpy,
        vehicle: Vehicle,
        gfx_update_time: float = 0.0,
        physics_update_time: float = 0.01,
        is_send_immediately: bool = False,
        is_visualised: bool = False,
    ):
        super().__init__(bng, vehicle)

        self.logger = getLogger(f"{LOGGER_ID}.RoadsSensor")
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.name = name
        self.vehicle = vehicle
        self.is_send_immediately = is_send_immediately
        self.is_visualised = is_visualised

        # Create and initialise this sensor in the simulation.
        self._open_roads_sensor(
            name,
            vehicle,
            gfx_update_time,
            physics_update_time,
            is_send_immediately,
            is_visualised,
        )

        # Fetch the unique Id number (in the simulator) for this roads sensor.  We will need this later.
        self.sensor_id = self._get_id()
        self.logger.debug("roadsSensor created: " f"{self.name}")

    def remove(self) -> None:
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_roads_sensor()
        self.logger.debug("roadsSensor removed: " f"{self.name}")

    def poll(self) -> StrDict:
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary containing the sensor readings data.

            The Roads sensor outputs:
            time: the time-stamp of the sample reading.
            dist2CL: the minimum distance between the vehicle front-axle-midpoint and road reference line (center line), in metres.
            dist2Left: the minimum distance between the vehicle front-axle-midpoint and left road edge, in metres.
            dist2Right: the minimum distance between vehicle front-axle-midpoint and right road edge, in metres.
            halfWidth: the half-width (center to edge) of the road at the front-axle-midpoint, in metres.
            roadRadius: the radius of the curvature of the road, in metres. (If the road is straight, then radius = NaN.)
            headingAngle: the angle between the vehicle forward direction and the road reference line, in rad.

            Coordinates of the coordinates of the four closest points (P0, P1, P2, P3) on the centerline of the road:
            xP0onCL, yP0onCL, zP0onCL: the world-space X, Y, Z coordinates of the closest road point, 'P0', in metres.
            xP1onCL, yP1onCL, zP1onCL: the world-space X, Y, Z coordinates of the 2nd-closest road point, 'P1', in metres.
            xP2onCL, yP2onCL, zP2onCL: the world-space X, Y, Z coordinates of the 3rd-closest road point, 'P2', in metres.
            xP3onCL, yP3onCL, zP3onCL: the world-space X, Y, Z coordinates of the 4th-closest road point, 'P3', in metres.

            The cubic parametric polynomial (U and V equations) of the centerline and the left and right roadedges:
            uAofCL, uBofCL, uCofCL, uDofCL: road Reference-Line Parametric Cubic Polynomial U equation; constant, linear, quadratic and cubic term.
            vAofCL, vBofCL, vCofCL, vDofCL: road Reference-Line Parametric Cubic Polynomial V equation; constant, linear, quadratic and cubic term.
            uAofLeftRE, uBofLeftRE, uCofLeftRE, uDofLeftRE: left roadedge Parametric Cubic Polynomial U equation; constant, linear, quadratic and cubic term.
            vAofLeftRE, vBofLeftRE, vCofLeftRE, vDofLeftRE: left roadedge Parametric Cubic Polynomial V equation; constant, linear, quadratic and cubic term.
            uAofRightRE, uBofRightRE, uCofRightRE, uDofRightRE: right roadedge Parametric Cubic Polynomial U equation; constant, linear, quadratic and cubic term.
            vAofRightRE, vBofRightRE, vCofRightRE, vDofRightRE: right roadedge Parametric Cubic Polynomial V equation; constant, linear, quadratic and cubic term.

            start points on the road:
            xStartCL, yStartCL, zStartCL: the starting point of the road centerline.
            xStartL, yStartL, zStartL: the coordinates of the starting point of the left roadedge (estimated).
            xStartR, yStartR, zStartR: the coordinates of the starting point of the right roadedge (estimated).

            semantic road data:
            drivability: the 'drivability' number of the road, where smaller = dirt/country roads and larger = highways etc.
            speedLimit: the speed limit of the road, in m/s.
            flag1way: a flag which indicates if the road is bi-directional (val = 0.0), or one-way (val = 1.0).
            numlane: number of lanes in the current travel direction.
        """
        # Send and receive a request for readings data from this sensor.
        readings_data = []
        if self.is_send_immediately:
            # Get the most-recent single reading from vlua.
            readings_data = self._poll_roads_sensor_VE()
        else:
            # Get the bulk data from ge lua.
            readings_data = self._poll_roads_sensor_GE()

        self.logger.debug(
            "roadsSensor readings received from simulation: " f"{self.name}"
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
        self.logger.debug("roadsSensor - ad-hoc polling request sent: " f"{self.name}")
        return int(
            self.send_recv_ge(
                "SendAdHocRequestRoadsSensor", name=self.name, vid=self.vehicle.vid
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
            "roadsSensor - ad-hoc polling request checked for completion: "
            f"{self.name}"
        )
        return self.send_recv_ge(
            "IsAdHocPollRequestReadyRoadsSensor", requestId=request_id
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
            "CollectAdHocPollRequestRoadsSensor", requestId=request_id
        )["data"]
        self.logger.debug(
            "roadsSensor - ad-hoc polling request returned and processed: "
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
            "SetRoadsSensorRequestedUpdateTime",
            ack="CompletedSetRoadsSensorRequestedUpdateTime",
            name=self.name,
            vid=self.vehicle.vid,
            GFXUpdateTime=requested_update_time,
        )

    def _get_id(self) -> int:
        return int(self.send_recv_ge("GetRoadsSensorId", name=self.name)["data"])

    def _open_roads_sensor(
        self,
        name: str,
        vehicle: Vehicle,
        gfx_update_time: float,
        physics_update_time: float,
        is_send_immediately: bool,
        is_visualised: bool,
    ) -> None:
        data: StrDict = dict()
        data["name"] = name
        data["vid"] = vehicle.vid
        data["GFXUpdateTime"] = gfx_update_time
        data["physicsUpdateTime"] = physics_update_time
        data["isSendImmediately"] = is_send_immediately
        data["isVisualised"] = is_visualised
        self.send_ack_ge(type="OpenRoadsSensor", ack="OpenedRoadsSensor", **data)
        self.logger.info(f'Opened RoadsSensor: "{name}"')

    def _close_roads_sensor(self) -> None:
        self.send_ack_ge(
            type="CloseRoadsSensor",
            ack="ClosedRoadsSensor",
            name=self.name,
            vid=self.vehicle.vid,
        )
        self.logger.info(f'Closed roadsSensor: "{self.name}"')

    def _poll_roads_sensor_GE(self) -> StrDict:
        return self.send_recv_ge("PollRoadsSensorGE", name=self.name)["data"]

    def _poll_roads_sensor_VE(self) -> StrDict:
        return self.send_recv_veh(
            "PollRoadsSensorVE", name=self.name, sensorId=self.sensor_id
        )["data"]
