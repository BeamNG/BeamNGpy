from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID
from beamngpy.types import Float3, StrDict

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

__all__ = ["AdvancedIMU"]


class AdvancedIMU(CommBase):
    """
    An interactive, automated IMU sensor, which produces regular acceleration and gyroscopic measurements in a local coordinate space.
    This sensor must be attached to a vehicle; it cannot be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
    A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
    will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.

    Args:
        name: A unique name for this advanced IMU sensor.
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this sensor should be attached. Note: a vehicle must be provided for the advanced IMU sensor.
        gfx_update_time: The gfx-step time which should pass between sensor reading updates to the user, in seconds.
        physics_update_time: The physics-step time which should pass between actual sampling the sensor, in seconds.
        pos: (X, Y, Z) Coordinate triplet specifying the position of the sensor, in world space.
        dir: (X, Y, Z) Coordinate triplet specifying the forward direction of the sensor.
        up: (X, Y, Z) Coordinate triplet specifying the up direction of the sensor.
        accel_window_width: The width of the window used in smoothing the acceleration data, if required.
        accel_frequency_cutoff: The filtering cutoff frequency to be used for acceleration (instead of a window width), if required.
        gyro_window_width: The width of the window used in smoothing the gyroscopic data, if required.
        gyro_frequency_cutoff: The filtering cutoff frequency to be used for gyroscopic (instead of a window width), if required.
        is_send_immediately: A flag which indicates if the readings should be sent back as soon as available or upon graphics step updates, as bulk.
        is_using_gravity: A flag which indicates whether this sensor should consider acceleration due to gravity in its computations, or not.
        is_allow_wheel_nodes: When using 'snap' attachment, this will allow sensors to be attached to vehicle wheels, and will rotate as the wheel turn.
        is_visualised: Whether or not to render the ultrasonic sensor points in the simulator.
        is_snapping_desired: A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle.
        is_force_inside_triangle: A flag which indicates if the sensor should be forced inside the nearest vehicle triangle.
        is_dir_world_space: Flag which indicates if the direction is provided in world-space coordinates (True), or the default vehicle space (False).
    """

    def __init__(
        self,
        name: str,
        bng: BeamNGpy,
        vehicle: Vehicle,
        gfx_update_time: float = 0.0,
        physics_update_time: float = 0.01,
        pos: Float3 = (0, 0, 1.7),
        dir: Float3 = (0, -1, 0),
        up: Float3 = (-0, 0, 1),
        accel_window_width: float | None = None,
        gyro_window_width: float | None = None,
        accel_frequency_cutoff: float | None = None,
        gyro_frequency_cutoff: float | None = None,
        is_send_immediately: bool = False,
        is_using_gravity: bool = False,
        is_allow_wheel_nodes: bool = True,
        is_visualised: bool = True,
        is_snapping_desired: bool = False,
        is_force_inside_triangle: bool = False,
        is_dir_world_space: bool = False,
    ):
        super().__init__(bng, vehicle)

        self.logger = getLogger(f"{LOGGER_ID}.Advanced IMU")
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.name = name
        self.is_send_immediately = is_send_immediately
        self.vehicle = vehicle

        # Create and initialise this sensor in the simulation.
        self._open_advanced_IMU(
            name,
            vehicle,
            gfx_update_time,
            physics_update_time,
            pos,
            dir,
            up,
            accel_window_width,
            gyro_window_width,
            is_send_immediately,
            accel_frequency_cutoff,
            gyro_frequency_cutoff,
            is_using_gravity,
            is_allow_wheel_nodes,
            is_visualised,
            is_snapping_desired,
            is_force_inside_triangle,
            is_dir_world_space,
        )

        # Fetch the unique Id number (in the simulator) for this advanced IMU sensor.  We will need this later.
        self.sensorId = self._get_advanced_imu_id()

        self.logger.debug("Advanced IMU - sensor created: " f"{self.name}")

    def remove(self) -> None:
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_advanced_IMU()
        self.logger.debug("Advanced IMU - sensor removed: " f"{self.name}")

    def poll(self) -> StrDict:
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary containing the sensor readings data.  Depending on the set poll timings, there may be multiple readings.  The data in each reading, by key, is as follows:
            time: the time of the reading, in seconds.
            mass: the local mass at the sensor position, in kg.
            accRaw: the raw (unsmoothed) acceleration data, for each of the three sensor axes, in ms^-2.
            accSmooth: the smoothed acceleration data, for each of the three sensor axes, in ms^-2.
            angVel: the raw (unsmoothed) angular velocity (rotational velocity), for each of the three sensor axes, in rad/s.
            angVelSmooth: the smoothed angular velocity (rotational velocity), for each of the three sensor axes, in rad/s.
            pos: the world-space position of this sensor, at the time of the reading, in meters * 3.
            dirX: the world-space direction vector of the sensors first axis (the sensor forward direction), upon which the acceleration and gyroscopic data was measured.
            dirY: the world-space direction vector of the sensors second axis (the sensor up direction), upon which the acceleration and gyroscopic data was measured.
            dirZ: the world-space direction vector of the sensors third axis, upon which the acceleration and gyroscopic data was measured.
        """
        # Send and receive a request for readings data from this sensor.
        readings_data = []
        if self.is_send_immediately:
            # Get the most-recent single reading from vlua.
            readings_data = self._poll_advanced_IMU_VE()
        else:
            # Get the bulk data from ge lua.
            readings_data = self._poll_advanced_IMU_GE()

        self.logger.debug(
            "Advanced IMU - sensor readings received from simulation: " f"{self.name}"
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
        self.logger.debug("Advanced IMU - ad-hoc polling request sent: " f"{self.name}")
        return int(
            self.send_recv_ge(
                "SendAdHocRequestAdvancedIMU", name=self.name, vid=self.vehicle.vid
            )["data"]
        )

    def is_ad_hoc_poll_request_ready(self, request_id: int) -> bool:
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug(
            "Advanced IMU - ad-hoc polling request checked for completion: "
            f"{self.name}"
        )
        return self.send_recv_ge(
            "IsAdHocPollRequestReadyAdvancedIMU", requestId=request_id
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
            "CollectAdHocPollRequestAdvancedIMU", requestId=request_id
        )["data"]
        self.logger.debug(
            "Advanced IMU - ad-hoc polling request returned and processed: "
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
            "SetAdvancedIMURequestedUpdateTime",
            ack="CompletedSetAdvancedIMURequestedUpdateTime",
            name=self.name,
            vid=self.vehicle.vid,
            GFXUpdateTime=requested_update_time,
        )

    def set_is_using_gravity(self, is_using_gravity: bool) -> None:
        """
        Sets whether this sensor is to include gravity in the computation or not.

        Args:
            is_using_gravity: A flag which indicates if this sensor is to use gravity in the computation or not.
        """
        return self.send_ack_ge(
            "SetAdvancedIMUIsUsingGravity",
            ack="CompletedSetAdvancedIMUIsUsingGravity",
            name=self.name,
            vid=self.vehicle.vid,
            isUsingGravity=is_using_gravity,
        )

    def set_is_visualised(self, is_visualised: bool) -> None:
        """
        Sets whether this sensor is to be visualised or not.

        Args:
            is_visualised: A flag which indicates if this sensor is to be visualised or not.
        """
        self.send_ack_ge(
            "SetAdvancedIMUIsVisualised",
            ack="CompletedSetAdvancedIMUIsVisualised",
            name=self.name,
            vid=self.vehicle.vid,
            isVisualised=is_visualised,
        )

    def _get_advanced_imu_id(self) -> int:
        return int(self.send_recv_ge("GetAdvancedImuId", name=self.name)["data"])

    def _open_advanced_IMU(
        self,
        name: str,
        vehicle: Vehicle,
        gfx_update_time: float,
        physics_update_time: float,
        pos: Float3,
        dir: Float3,
        up: Float3,
        accel_window_width: float | None,
        gyro_window_width: float | None,
        is_send_immediately: bool,
        accel_frequency_cutoff: float | None,
        gyro_frequency_cutoff: float | None,
        is_using_gravity: bool,
        is_allow_wheel_nodes: bool,
        is_visualised: bool,
        is_snapping_desired: bool,
        is_force_inside_triangle: bool,
        is_dir_world_space: bool,
    ) -> None:
        data: StrDict = dict()
        data["name"] = name
        data["vid"] = vehicle.vid
        data["GFXUpdateTime"] = gfx_update_time
        data["physicsUpdateTime"] = physics_update_time
        data["pos"] = pos
        data["dir"] = dir
        data["up"] = up
        data["accelWindowWidth"] = accel_window_width
        data["accelFrequencyCutoff"] = accel_frequency_cutoff
        data["gyroWindowWidth"] = gyro_window_width
        data["gyroFrequencyCutoff"] = gyro_frequency_cutoff
        data["isSendImmediately"] = is_send_immediately
        data["isUsingGravity"] = is_using_gravity
        data["isAllowWheelNodes"] = is_allow_wheel_nodes
        data["isVisualised"] = is_visualised
        data["isSnappingDesired"] = is_snapping_desired
        data["isForceInsideTriangle"] = is_force_inside_triangle
        data["isDirWorldSpace"] = is_dir_world_space
        self.send_ack_ge(type="OpenAdvancedIMU", ack="OpenedAdvancedIMU", **data)
        self.logger.info(f'Opened advanced IMU sensor: "{name}"')

    def _close_advanced_IMU(self) -> None:
        self.send_ack_ge(
            type="CloseAdvancedIMU",
            ack="ClosedAdvancedIMU",
            name=self.name,
            vid=self.vehicle.vid,
        )
        self.logger.info(f'Closed advanced IMU sensor: "{self.name}"')

    def _poll_advanced_IMU_GE(self) -> StrDict:
        return self.send_recv_ge("PollAdvancedImuGE", name=self.name)["data"]

    def _poll_advanced_IMU_VE(self) -> StrDict:
        return self.send_recv_veh(
            "PollAdvancedImuVE", name=self.name, sensorId=self.sensorId
        )["data"]
