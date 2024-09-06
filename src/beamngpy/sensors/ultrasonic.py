from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID
from beamngpy.types import Float2, Float3, Int2, StrDict
import numpy as np

from beamngpy.sensors.shmem import BNGSharedMemory

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

__all__ = ["Ultrasonic"]


class Ultrasonic(CommBase):
    """
    An interactive, automated ultrasonic sensor, which produces regular distance measurements, ready for further processing.
    This sensor can be attached to a vehicle, or can be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
    A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
    will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.

    Args:
        name: A unique name for this ultrasonic sensor.
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this sensor should be attached, if any.
        requested_update_time: The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
        update_priority: The priority which the sensor should ask for new readings. lowest -> 0, highest -> 1.
        pos: (X, Y, Z) Coordinate triplet specifying the position of the sensor, in world space.
        dir: (X, Y, Z) Coordinate triplet specifying the forward direction of the sensor.
        up: (X, Y, Z) Coordinate triplet specifying the up direction of the sensor.
        size: (X, Y) The resolution of the sensor (the size of the depth buffer image in the distance measurement computation).
        field_of_view_y: The sensor vertical field of view parameters.
        near_far_planes: (X, Y) The sensor near and far plane distances.
        range_roundness: the general roudness of the ultrasonic sensor range-shape. Can be negative.
        range_cutoff_sensitivity: a cutoff sensitivity parameter for the ultrasonic sensor range-shape.
        range_shape: the shape of the ultrasonic sensor range-shape in [0, 1], from conical to circular.
        range_focus: the focus parameter for the ultrasonic sensor range-shape.
        range_min_cutoff: the minimum cut-off distance for the ultrasonic sensor range-shape. Nothing closer than this will be detected.
        range_direct_max_cutoff: the maximum cut-off distance for the ultrasonic sensor range-shape. This parameter is a hard cutoff - nothing
            further than this will be detected, although other parameters can also control the max distance.
        sensitivity: an ultrasonic sensor sensitivity parameter.
        fixed_window_size: an ultrasonic sensor sensitivity parameter.
        is_visualised: Whether or not to render the ultrasonic sensor points in the simulator.
        is_streaming: Whether or not to stream the data directly to shared memory (no poll required, for efficiency - BeamNGpy won't block.)
        is_static: A flag which indicates whether this sensor should be static (fixed position), or attached to a vehicle.
        is_snapping_desired: A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle (not used for static sensors).
        is_force_inside_triangle: A flag which indicates if the sensor should be forced inside the nearest vehicle triangle (not used for static sensors).
        is_dir_world_space: Flag which indicates if the direction is provided in world-space coordinates (True), or the default vehicle space (False).
    """

    def __init__(
        self,
        name: str,
        bng: BeamNGpy,
        vehicle: Vehicle | None = None,
        requested_update_time: float = 0.1,
        update_priority: float = 0.0,
        pos: Float3 = (0, 0, 1.7),
        dir: Float3 = (0, -1, 0),
        up: Float3 = (0, 0, 1),
        resolution: Int2 = (200, 200),
        field_of_view_y: float = 5.7,
        near_far_planes: Float2 = (0.1, 5.1),
        range_roundness: float = -1.15,
        range_cutoff_sensitivity: float = 0.0,
        range_shape: float = 0.3,
        range_focus: float = 0.376,
        range_min_cutoff: float = 0.1,
        range_direct_max_cutoff: float = 5.0,
        sensitivity: float = 3.0,
        fixed_window_size: float = 10,
        is_visualised: bool = True,
        is_streaming: bool = False,
        is_static: bool = False,
        is_snapping_desired: bool = False,
        is_force_inside_triangle: bool = False,
        is_dir_world_space: bool = False,
    ):
        super().__init__(bng, vehicle)

        self.logger = getLogger(f"{LOGGER_ID}.Ultrasonic")
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name

        # Shared memory for velocity data streaming.
        self.shmem_size = None
        self.shmem = None
        if is_streaming == True:
            self.shmem_size = 4
            self.shmem = BNGSharedMemory(self.shmem_size)

        # Create and initialise this sensor in the simulation.
        self._open_ultrasonic(
            name,
            vehicle,
            self.shmem.name if self.shmem else None,
            self.shmem_size,
            requested_update_time,
            update_priority,
            pos,
            dir,
            up,
            resolution,
            field_of_view_y,
            near_far_planes,
            range_roundness,
            range_cutoff_sensitivity,
            range_shape,
            range_focus,
            range_min_cutoff,
            range_direct_max_cutoff,
            sensitivity,
            fixed_window_size,
            is_visualised,
            is_streaming,
            is_static,
            is_snapping_desired,
            is_force_inside_triangle,
            is_dir_world_space,
        )
        self.logger.debug("Ultrasonic - sensor created: " f"{self.name}")

    def remove(self):
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_ultrasonic()
        self.logger.debug("Ultrasonic - sensor removed: " f"{self.name}")

    def poll(self) -> StrDict:
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary containing the following keys:

            distance: the latest distance measurement, in meters.
            windowMin: (internal parameter) which indicates the minimum size of the data filtering window used in post-processing. This can usually be ignored.
            windowMax: (internal parameter) which indicates the maximum size of the data filtering window used in post-processing. This can usually be ignored.
        """
        # Send and receive a request for readings data from this sensor.
        distance_measurement = self.send_recv_ge("PollUltrasonic", name=self.name)[
            "data"
        ]
        self.logger.debug(
            "Ultrasonic - sensor readings received from simulation: " f"{self.name}"
        )

        return distance_measurement

    def stream(self):
        """
        Gets the latest Ultrasonic distance reading from shared memory (which is being streamed directly).

        Returns:
            The latest Ultrasonic distance reading from shared memory.
        """
        return np.frombuffer(self.shmem.read(self.shmem_size), dtype=np.float32)

    def send_ad_hoc_poll_request(self) -> int:
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            A unique Id number for the ad-hoc request.
        """
        self.logger.debug("Ultrasonic - ad-hoc polling request sent: " f"{self.name}")
        return int(
            self.send_recv_ge("SendAdHocRequestUltrasonic", name=self.name)["data"]
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
            "Ultrasonic - ad-hoc polling request checked for completion: "
            f"{self.name}"
        )
        return self.send_recv_ge(
            "IsAdHocPollRequestReadyUltrasonic", requestId=request_id
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
            "CollectAdHocPollRequestUltrasonic", requestId=request_id
        )["data"]
        self.logger.debug(
            "Ultrasonic - ad-hoc polling request returned and processed: "
            f"{self.name}"
        )

        return readings

    def get_requested_update_time(self) -> float:
        """
        Gets the current 'requested update time' value for this sensor.

        Returns:
            (float): The requested update time.
        """
        return self.send_recv_ge("GetUltrasonicRequestedUpdateTime", name=self.name)[
            "data"
        ]

    def get_update_priority(self) -> float:
        """
        Gets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, highest to lowest.

        Returns:
            The update priority value.
        """
        return self.send_recv_ge("GetUltrasonicUpdatePriority", name=self.name)["data"]

    def get_position(self) -> Float3:
        """
        Gets the current world-space position of this sensor.

        Returns:
            The sensor position.
        """
        table = self.send_recv_ge("GetUltrasonicSensorPosition", name=self.name)["data"]
        return (table["x"], table["y"], table["z"])

    def get_direction(self) -> Float3:
        """
        Gets the current direction vector of this sensor.

        Returns:
            The sensor direction.
        """
        table = self.send_recv_ge("GetUltrasonicSensorDirection", name=self.name)[
            "data"
        ]
        return (table["x"], table["y"], table["z"])

    def get_max_pending_requests(self) -> int:
        """
        Gets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Returns:
            The max pending requests value.
        """
        return int(
            self.send_recv_ge("GetUltrasonicMaxPendingGpuRequests", name=self.name)[
                "data"
            ]
        )

    def get_is_visualised(self) -> bool:
        """
        Gets a flag which indicates if this ultrasonic sensor is visualised or not.

        Returns:
            A flag which indicates if this ultrasonic sensor is visualised or not.
        """
        return self.send_recv_ge("GetUltrasonicIsVisualised", name=self.name)["data"]

    def set_requested_update_time(self, requested_update_time: float):
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time: The new requested update time.
        """
        return self.send_ack_ge(
            "SetUltrasonicRequestedUpdateTime",
            ack="CompletedSetUltrasonicRequestedUpdateTime",
            name=self.name,
            updateTime=requested_update_time,
        )

    def set_update_priority(self, update_priority: float) -> None:
        """
        Sets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, , highest to lowest.

        Args:
            update_priority: The new update priority
        """
        return self.send_ack_ge(
            "SetUltrasonicUpdatePriority",
            ack="CompletedSetUltrasonicUpdatePriority",
            name=self.name,
            updatePriority=update_priority,
        )

    def set_max_pending_requests(self, max_pending_requests: int) -> None:
        """
        Sets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Args:
            max_pending_requests: The new max pending requests value.
        """
        self.send_ack_ge(
            "SetUltrasonicMaxPendingGpuRequests",
            ack="CompletedSetUltrasonicMaxPendingGpuRequests",
            name=self.name,
            maxPendingGpuRequests=max_pending_requests,
        )

    def set_is_visualised(self, is_visualised: bool) -> None:
        """
        Sets whether this ultrasonic sensor is to be visualised or not.

        Args:
            is_visualised: A flag which indicates if this ultrasonic sensor is to be visualised or not.
        """
        self.send_ack_ge(
            "SetUltrasonicIsVisualised",
            ack="CompletedSetUltrasonicIsVisualised",
            name=self.name,
            isVisualised=is_visualised,
        )

    def _open_ultrasonic(
        self,
        name: str,
        vehicle: Vehicle | None,
        shmem_name: str | None,
        shmem_size: int,
        requested_update_time: float,
        update_priority: float,
        pos: Float3,
        dir: Float3,
        up: Float3,
        size: Int2,
        field_of_view_y: float,
        near_far_planes: Float2,
        range_roundness: float,
        range_cutoff_sensitivity: float,
        range_shape: float,
        range_focus: float,
        range_min_cutoff: float,
        range_direct_max_cutoff: float,
        sensitivity: float,
        fixed_window_size: float,
        is_visualised: bool,
        is_streaming: bool,
        is_static: bool,
        is_snapping_desired: bool,
        is_force_inside_triangle: bool,
        is_dir_world_space: bool,
    ) -> None:
        data: StrDict = dict()
        data["name"] = name
        data["shmemHandle"] = shmem_name
        data["shmemSize"] = shmem_size
        data["vid"] = 0
        if vehicle is not None:
            data["vid"] = vehicle.vid
        data["updateTime"] = requested_update_time
        data["priority"] = update_priority
        data["pos"] = pos
        data["dir"] = dir
        data["up"] = up
        data["size"] = size
        data["fovY"] = field_of_view_y
        data["near_far_planes"] = near_far_planes
        data["range_roundness"] = range_roundness
        data["range_cutoff_sensitivity"] = range_cutoff_sensitivity
        data["range_shape"] = range_shape
        data["range_focus"] = range_focus
        data["range_min_cutoff"] = range_min_cutoff
        data["range_direct_max_cutoff"] = range_direct_max_cutoff
        data["sensitivity"] = sensitivity
        data["fixed_window_size"] = fixed_window_size
        data["isVisualised"] = is_visualised
        data["isStreaming"] = is_streaming
        data["isStatic"] = is_static
        data["isSnappingDesired"] = is_snapping_desired
        data["isForceInsideTriangle"] = is_force_inside_triangle
        data["isDirWorldSpace"] = is_dir_world_space

        self.send_ack_ge("OpenUltrasonic", ack="OpenedUltrasonic", **data)
        self.logger.info(f'Opened ultrasonic sensor: "{name}"')

    def _close_ultrasonic(self) -> None:
        self.send_ack_ge("CloseUltrasonic", ack="ClosedUltrasonic", name=self.name)
        self.logger.info(f'Closed ultrasonic sensor: "{self.name}"')
