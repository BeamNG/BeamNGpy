from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

import numpy as np

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID
from beamngpy.sensors.shmem import BNGSharedMemory
from beamngpy.types import Float3, StrDict

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

# The maximum number of LiDAR points which can be used.
# TODO: Make this more efficient by instead computing the number of LiDAR points based on the sensor parameter values.
MAX_LIDAR_POINTS = 2000000


class Lidar(CommBase):
    """
    An automated LiDAR sensor, which produces raw LiDAR point clouds, ready for further processing by the user.

    This sensor runs in various modes, which are chosen using the two flag arguments 'is_rotate_mode' and 'is_360_mode':

    ** MODE I: Full 360 Degrees Mode: **
        The LiDAR will return a point cloud covering the full 360 degree sector around the provided 'up' vector, with the chosen parameters.
        This mode should be used if fast, 360 degree updates are required.  For fastest results, use shared memory or see the 'stream()' functions.
        Note: There is no need to provide a horizontal angle for this mode, since it will be fixed upon instantiation.
        Likewise, the direction vector is meaningless when using this mode, since the full 360 degrees will be scanned with every reading.
        [For this mode, leave the flags as default or switch is_360_mode=True, is_rotate_mode=False].

    ** MODE II: LFO Mode: **
        The LiDAR will operate in a low-frequency rotating mode and provide readings local to the sector at which it currently points.
        For best results, the frequency should be in the range [1Hz, 8Hz].  Faster frequencies (on this mode) may cause undersampling in some sectors, so beware!
        The horizontal angle defines the aperture of the returns in the currently-facing direction, as the LiDAR spins.
        The direction vector will specify the starting direction only, for this mode.
        [For this mode, set the flags as follows: is_360_mode=False, is_rotate_mode=True].

    ** MODE III: Static Mode: **
        The LiDAR will operate in a fixed static pose relative to the vehicle (ie it doesn't rotate). You can set the horizontal angle and direction and this
        will remain so for the lifetime of the sensor.  The horizontal angle is supported in the range [1, 179] degrees (the full hemisphere, basically).
        [For this mode, set the flags as follows: is_360_mode=False, is_rotate_mode=False].

    All three LiDAR modes also support the return of semantic annotations (segmentations).  This can be switched on with the similarly-named flag (see below).

    Shared memory can be used with all modes, if required.  This will be much faster than sending big LiDAR point clouds over the TCP socket, and is
    recommended (however, not everyone will be running BeamNGPy on the same machine as BeamNG.Tech, so this may not be an option).  See the provided 'stream()'
    functions for maximum speed here.

    This sensor can be attached to a vehicle, or can be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
    A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
    will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.

    Args:
        name: A unique name for this LiDAR sensor.
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this sensor should be attached, if any.
        requested_update_time: The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
        update_priority: The priority which the sensor should ask for new readings. lowest -> 0, highest -> 1.
        pos: (X, Y, Z) coordinate triplet specifying the position of the sensor, in world space.
        dir: (X, Y, Z) Coordinate triplet specifying the forward direction of the sensor.
        up: (X, Y, Z) Coordinate triplet specifying the up direction of the sensor.
        vertical_resolution: The vertical resolution of this LiDAR sensor.
        vertical_angle: The vertical angle of this LiDAR sensor, in degrees.
        frequency: The frequency of this LiDAR sensor.
        horizontal_angle: The horizontal angle of this LiDAR sensor.
        max_distance: The maximum distance which this LiDAR sensor will detect, in metres.
        is_rotate_mode: Runs the LiDAR sensor in 'LFO rotate'.  Should be used with frequencies in the range [1Hz - 10Hz, or so, for best results].
        is_360_mode: Runs the LiDAR sensor in 'Full 360 Degrees' mode. Note: there is no need to provide a horizontal angle for this mode.
        is_using_shared_memory: A flag which indicates if we should use shared memory to send/recieve the sensor readings data.
        is_visualised: A flag which indicates if this LiDAR sensor should appear visualised or not.
        is_streaming: Whether or not to stream the data directly to shared memory (no poll required, for efficiency - BeamNGpy won't block.)
        is_annotated: A flag which indicates if this LiDAR sensor should return annotation data instead of distance data.
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
        vertical_resolution: int = 64,
        vertical_angle: float = 26.9,
        frequency: float = 20,
        horizontal_angle: float = 360,
        max_distance: float = 120,
        is_rotate_mode: bool = False,
        is_360_mode: bool = True,
        is_using_shared_memory: bool = True,
        is_visualised: bool = True,
        is_streaming: bool = False,
        is_annotated: bool = False,
        is_static: bool = False,
        is_snapping_desired: bool = False,
        is_force_inside_triangle: bool = False,
        is_dir_world_space: bool = False,
    ):
        super().__init__(bng, vehicle)

        self.logger = getLogger(f"{LOGGER_ID}.Lidar")
        self.logger.setLevel(DEBUG)

        self.name = name

        # Set up the shared memory for this sensor, if requested.
        self.is_using_shared_memory = is_using_shared_memory
        self.is_streaming = is_streaming
        self.point_cloud_shmem_size = MAX_LIDAR_POINTS * 3 * 4
        self.point_cloud_shmem: BNGSharedMemory | None = None
        self.colour_shmem_size = MAX_LIDAR_POINTS * 4
        self.colour_shmem: BNGSharedMemory | None = None
        if is_using_shared_memory:
            self.point_cloud_shmem = BNGSharedMemory(self.point_cloud_shmem_size)
            self.logger.debug(
                f"Lidar - Bound shared memory for point cloud data: {self.point_cloud_shmem.name}"
            )

            self.colour_shmem = BNGSharedMemory(self.colour_shmem_size)
            self.logger.debug(
                f"Lidar - Bound shared memory for colour data: {self.colour_shmem.name}"
            )

        # Create and initialise this sensor in the simulation.
        point_cloud_shmem_name = (
            self.point_cloud_shmem.name if self.point_cloud_shmem else None
        )
        colour_shmem_name = self.colour_shmem.name if self.colour_shmem else None
        self._open_lidar(
            name,
            vehicle,
            is_using_shared_memory,
            point_cloud_shmem_name,
            self.point_cloud_shmem_size,
            colour_shmem_name,
            self.colour_shmem_size,
            requested_update_time,
            update_priority,
            pos,
            dir,
            up,
            vertical_resolution,
            vertical_angle,
            frequency,
            horizontal_angle,
            max_distance,
            is_rotate_mode,
            is_360_mode,
            is_visualised,
            is_streaming,
            is_annotated,
            is_static,
            is_snapping_desired,
            is_force_inside_triangle,
            is_dir_world_space,
        )
        self.logger.debug("Lidar - sensor created: " f"{self.name}")

    def _convert_binary_to_array(self, binary: StrDict) -> StrDict:
        """
        Converts the binary string data from the simulator, which contains the point cloud and colour data, into arrays.

        Args:
            binary: The raw readings data, as a binary string.
        Returns:
            A dictionary containing the point cloud and colour data.
        """
        processed_readings: StrDict = dict(type="Lidar")

        if len(binary["pointCloud"]) == 0:
            processed_readings["pointCloud"] = np.empty(0, dtype=np.float32)
            processed_readings["colours"] = np.empty(0, dtype=np.uint8)
            return processed_readings

        # Format the point cloud data.
        floats = np.frombuffer(binary["pointCloud"], dtype=np.float32)
        if self.is_streaming:
            n_points = int(floats[-1])
            floats = floats[: 3 * n_points]
        processed_readings["pointCloud"] = floats.reshape((-1, 3)).copy()

        # Format the corresponding colour data.
        colours = np.frombuffer(binary["colours"], dtype=np.uint8)
        if self.is_streaming:
            colours = colours[: 4 * n_points]
        processed_readings["colours"] = colours.reshape((-1, 4)).copy()  # rgba

        return processed_readings

    def remove(self) -> None:
        """
        Removes this sensor from the simulation.
        """
        # Remove the shared memory binding being used by this sensor, if applicable.
        if self.is_using_shared_memory:
            assert self.point_cloud_shmem
            self.logger.debug(
                "Lidar - Unbinding shared memory: " f"{self.point_cloud_shmem.name}"
            )
            self.point_cloud_shmem.try_close()

            assert self.colour_shmem
            self.logger.debug(
                "Lidar - Unbinding shared memory: " f"{self.colour_shmem.name}"
            )
            self.colour_shmem.try_close()

        # Remove this sensor from the simulation.
        self._close_lidar()
        self.logger.debug("Lidar - sensor removed: " f"{self.name}")

    def poll_raw(self):
        """
        Gets the most-recent readings for this sensor as unprocessed bytes.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary with values being the unprocessed bytes representing the RGBA data from the sensors and
            the following keys

            * ``pointCloud``: The colour data.
            * ``colours``: The semantic annotation data.
        """
        if self.is_using_shared_memory:
            raw_readings = {}
            if not self.is_streaming:
                sizes = self.send_recv_ge(
                    "PollLidar",
                    name=self.name,
                    isUsingSharedMemory=self.is_using_shared_memory,
                )["data"]
            assert self.point_cloud_shmem
            raw_readings["pointCloud"] = self.point_cloud_shmem.read(
                self.point_cloud_shmem_size
            )
            self.logger.debug(
                "Lidar - point cloud data read from shared memory: " f"{self.name}"
            )

            assert self.colour_shmem
            raw_readings["colours"] = self.colour_shmem.read(self.colour_shmem_size)
            self.logger.debug(
                "Lidar - colour data read from shared memory: " f"{self.name}"
            )

            if not self.is_streaming:
                raw_readings["pointCloud"] = raw_readings["pointCloud"][
                    : int(sizes["points"])
                ]
                raw_readings["colours"] = raw_readings["colours"][
                    : int(sizes["colours"])
                ]
        else:
            raw_readings = self.send_recv_ge(
                "PollLidar",
                name=self.name,
                isUsingSharedMemory=self.is_using_shared_memory,
            )["data"]
            self.logger.debug("Lidar - LiDAR data read from socket: " f"{self.name}")
        return raw_readings

    def poll(self) -> StrDict:
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary with the following keys:

            * ``pointCloud``: The point cloud readings, as a dictionary of vectors.
            * ``colours``: The semantic annotation data, as a dictionary of colours for each corresponding point in the point cloud.
        """
        processed_readings: StrDict = dict(type="Lidar")

        # Get the LiDAR point cloud and colour data, and format it before returning it.
        raw_readings = self.poll_raw()
        processed_readings = self._convert_binary_to_array(raw_readings)
        return processed_readings

    def stream(self) -> StrDict:
        """
        Gets the streamed LiDAR point cloud data from the associated shared memory location.

        Returns:
            The LiDAR point cloud data.
        """
        return self.poll()

    # The following three functions are used together to send and recieve single 'ad-hoc' style sensor requests.
    # This is for users who only want occasional readings now and again, which they can request, wait for, then collect later.

    def send_ad_hoc_poll_request(self) -> int:
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            A unique Id number for the ad-hoc request.
        """
        self.logger.debug("Lidar - ad-hoc polling request sent: " f"{self.name}")
        return int(self.send_recv_ge("SendAdHocRequestLidar", name=self.name)["data"])

    def is_ad_hoc_poll_request_ready(self, request_id: int) -> bool:
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug(
            "Lidar - ad-hoc polling request checked for completion: " f"{self.name}"
        )
        return self.send_recv_ge("IsAdHocPollRequestReadyLidar", requestId=request_id)[
            "data"
        ]

    def collect_ad_hoc_poll_request(self, request_id: int) -> StrDict:
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.
        Returns:
            A dictionary containing the LiDAR point cloud and colour data.
        """
        # Get the binary string data from the simulator.
        binary = self.send_recv_ge(
            "CollectAdHocPollRequestLidar", requestId=request_id
        )["data"]

        self.logger.debug("Lidar - LiDAR data read from socket: " f"{self.name}")
        return self._convert_binary_to_array(binary)

    def get_requested_update_time(self) -> float:
        """
        Gets the current 'requested update time' value for this sensor.

        Returns:
            The requested update time.
        """
        return self.send_recv_ge("GetLidarRequestedUpdateTime", name=self.name)["data"]

    def get_update_priority(self) -> float:
        """
        Gets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, highest to lowest.

        Returns:
            The update priority value.
        """
        return self.send_recv_ge("GetLidarUpdatePriority", name=self.name)["data"]

    def get_position(self) -> Float3:
        """
        Gets the current world-space position of this sensor.

        Returns:
            The sensor position.
        """
        table = self.send_recv_ge("GetLidarSensorPosition", name=self.name)["data"]
        return (table["x"], table["y"], table["z"])

    def get_direction(self) -> Float3:
        """
        Gets the current direction vector of this sensor.

        Returns:
            The sensor direction.
        """
        table = self.send_recv_ge("GetLidarSensorDirection", name=self.name)["data"]
        return (table["x"], table["y"], table["z"])

    def get_max_pending_requests(self) -> int:
        """
        Gets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Returns:
            The max pending requests value.
        """
        return int(
            self.send_recv_ge("GetLidarMaxPendingGpuRequests", name=self.name)["data"]
        )

    def get_is_visualised(self) -> bool:
        """
        Gets a flag which indicates if this LiDAR sensor is visualised or not.

        Returns:
            A flag which indicates if this LiDAR sensor is visualised or not.
        """
        return self.send_recv_ge("GetLidarIsVisualised", name=self.name)["data"]

    def get_is_annotated(self) -> bool:
        """
        Gets a flag which indicates if this LiDAR sensor is annotated or not.

        Returns:
            A flag which indicates if this LiDAR sensor is annotated or not.
        """
        return self.send_recv_ge("GetLidarIsAnnotated", name=self.name)["data"]

    def set_requested_update_time(self, requested_update_time: float) -> None:
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            update_priority: The new requested update time.
        """
        self.send_ack_ge(
            "SetLidarRequestedUpdateTime",
            ack="CompletedSetLidarRequestedUpdateTime",
            name=self.name,
            updateTime=requested_update_time,
        )

    def set_update_priority(self, update_priority: float) -> None:
        """
        Sets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, , highest to lowest.

        Args:
            update_priority: The new update priority value.
        """
        self.send_ack_ge(
            "SetLidarUpdatePriority",
            ack="CompletedSetLidarUpdatePriority",
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
            "SetLidarMaxPendingGpuRequests",
            ack="CompletedSetLidarMaxPendingGpuRequests",
            name=self.name,
            maxPendingGpuRequests=max_pending_requests,
        )

    def set_is_visualised(self, is_visualised: bool) -> None:
        """
        Sets whether this LiDAR sensor is to be visualised or not.

        Args:
            is_visualised: A flag which indicates if this LiDAR sensor is to be visualised or not.
        """
        self.send_ack_ge(
            "SetLidarIsVisualised",
            ack="CompletedSetLidarIsVisualised",
            name=self.name,
            isVisualised=is_visualised,
        )

    def set_is_annotated(self, is_annotated: bool) -> None:
        """
        Sets whether this LiDAR sensor is to be annotated or not. This means it will return annotation data instead of distances.

        Args:
            is_annotated: A flag which indicates if this LiDAR sensor is to be annotated or not.
        """
        self.send_ack_ge(
            "SetLidarIsAnnotated",
            ack="CompletedSetLidarIsAnnotated",
            name=self.name,
            isAnnotated=is_annotated,
        )

    def _open_lidar(
        self,
        name: str,
        vehicle: Vehicle | None,
        is_using_shared_memory: bool,
        point_cloud_shmem_name: str | None,
        point_cloud_shmem_size: int,
        colour_shmem_name: str | None,
        colour_shmem_size: int,
        requested_update_time: float,
        update_priority: float,
        pos: Float3,
        dir: Float3,
        up: Float3,
        vertical_resolution: int,
        vertical_angle: float,
        frequency: float,
        horizontal_angle: float,
        max_distance: float,
        is_rotate_mode: bool,
        is_360_mode: bool,
        is_visualised: bool,
        is_streaming: bool,
        is_annotated: bool,
        is_static: bool,
        is_snapping_desired: bool,
        is_force_inside_triangle: bool,
        is_dir_world_space: bool,
    ):
        data: StrDict = dict()
        data["vid"] = 0
        if vehicle is not None:
            data["vid"] = vehicle.vid
        data["useSharedMemory"] = is_using_shared_memory
        data["name"] = name
        data["pointCloudShmemHandle"] = point_cloud_shmem_name
        data["pointCloudShmemSize"] = point_cloud_shmem_size
        data["colourShmemHandle"] = colour_shmem_name
        data["colourShmemSize"] = colour_shmem_size
        data["updateTime"] = requested_update_time
        data["priority"] = update_priority
        data["pos"] = pos
        data["dir"] = dir
        data["up"] = up
        data["vRes"] = vertical_resolution
        data["vAngle"] = vertical_angle
        data["hz"] = frequency
        data["hAngle"] = horizontal_angle
        data["maxDist"] = max_distance
        data["isRotate"] = is_rotate_mode
        data["is360"] = is_360_mode
        data["isVisualised"] = is_visualised
        data["isStreaming"] = is_streaming
        data["isAnnotated"] = is_annotated
        data["isStatic"] = is_static
        data["isSnappingDesired"] = is_snapping_desired
        data["isForceInsideTriangle"] = is_force_inside_triangle
        data["isDirWorldSpace"] = is_dir_world_space
        self.send_ack_ge(type="OpenLidar", ack="OpenedLidar", **data)
        self.logger.info(f'Opened lidar: "{name}"')

    def _close_lidar(self) -> None:
        """
        Closes the Lidar instance.
        """
        self.send_ack_ge(type="CloseLidar", ack="ClosedLidar", name=self.name)
        self.logger.info(f'Closed lidar: "{self.name}"')
