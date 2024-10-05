from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID
from beamngpy.types import Float2, Float3, Int2, StrDict

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

import math
import struct

import matplotlib.pyplot as plt
import numpy as np

from beamngpy.sensors.shmem import BNGSharedMemory

__all__ = ["Radar"]


class Radar(CommBase):
    """
    An interactive, automated RADAR sensor, which produces regular RADAR measurements.
    This sensor can be attached to a vehicle, or can be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
    A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
    will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.

    Args:
        name: A unique name for this RADAR sensor.
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this sensor should be attached, if any.
        requested_update_time: The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
        update_priority: The priority which the sensor should ask for new readings. lowest -> 0, highest -> 1.
        pos: (X, Y, Z) Coordinate triplet specifying the position of the sensor, in world space.
        dir: (X, Y, Z) Coordinate triplet specifying the forward direction of the sensor.
        up: (X, Y, Z) Coordinate triplet specifying the up direction of the sensor.
        range_bins: The number of bins to use in the range dimension, for RADAR post-processing (the images returned from the simulator).
        azimuth_bins: The number of bins to use in the azimuth dimension, for RADAR post-processing (PPI plots).
        vel_bins: The number of bins to use in the velocity dimension, for RADAR post-processing (range-Doppler plots).
        range_min: The minimum range to display in the post-processing.
        range_max: The maximum range to display in the post-processing.
        vel_min: The minimum velocity to display in the post-processing (range-Doppler images), in m/s.
        vel_max: The maximum velocity to display in the post-processing (range-Doppler images), in m/s.
        half_angle_deg: On the PPI plot, this is half the azimuthal range (angle between the vertical and cone edge), in degrees.
        size: (X, Y) The resolution of the sensor (the size of the depth buffer image in the distance measurement computation).
        field_of_view_y: The sensor vertical field of view parameter.
        near_far_planes: (X, Y) The sensor near and far plane distances.
        range_roundness: the general roudness of the RADAR sensor range-shape. Can be negative.
        range_cutoff_sensitivity: a cutoff sensitivity parameter for the RADAR sensor range-shape.
        range_shape: the shape of the RADAR sensor range-shape in [0, 1], from conical to circular.
        range_focus: the focus parameter for the RADAR sensor range-shape.
        range_min_cutoff: the minimum cut-off distance for the RADAR sensor range-shape. Nothing closer than this will be detected.
        range_direct_max_cutoff: the maximum cut-off distance for the RADAR sensor range-shape. This parameter is a hard cutoff - nothing
            further than this will be detected, although other parameters can also control the max distance.
        is_visualised: Whether or not to render the RADAR sensor points in the simulator.
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
        range_bins: int = 200,
        azimuth_bins: int = 200,
        vel_bins: int = 200,
        range_min: float = 0.1,
        range_max: float = 100.0,
        vel_min: float = -50.0,
        vel_max: float = 50.0,
        half_angle_deg: float = 30.0,
        resolution: Int2 = (200, 200),
        field_of_view_y: float = 70,
        near_far_planes: Float2 = (0.1, 150.0),
        range_roundness: float = -2.0,
        range_cutoff_sensitivity: float = 0.0,
        range_shape: float = 0.23,
        range_focus: float = 0.12,
        range_min_cutoff: float = 0.5,
        range_direct_max_cutoff: float = 150.0,
        is_visualised: bool = True,
        is_streaming: bool = False,
        is_static: bool = False,
        is_snapping_desired: bool = False,
        is_force_inside_triangle: bool = False,
        is_dir_world_space: bool = False,
    ):
        super().__init__(bng, vehicle)

        self.logger = getLogger(f"{LOGGER_ID}.RADAR")
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.name = name

        # Shared memory for velocity data streaming.
        self.shmem_size = 1000 * 1000 * 4
        self.shmem: BNGSharedMemory | None = None
        self.shmem2: BNGSharedMemory | None = None
        if is_streaming:
            self.shmem_size = 1000 * 1000 * 4
            self.shmem = BNGSharedMemory(self.shmem_size)
            self.shmem2 = BNGSharedMemory(self.shmem_size)

        # Create and initialise this sensor in the simulation.
        self._open_radar(
            name,
            vehicle,
            self.shmem.name if self.shmem else None,
            self.shmem2.name if self.shmem2 else None,
            self.shmem_size,
            requested_update_time,
            update_priority,
            pos,
            dir,
            up,
            range_bins,
            azimuth_bins,
            vel_bins,
            range_min,
            range_max,
            vel_min,
            vel_max,
            half_angle_deg,
            resolution,
            field_of_view_y,
            near_far_planes,
            range_roundness,
            range_cutoff_sensitivity,
            range_shape,
            range_focus,
            range_min_cutoff,
            range_direct_max_cutoff,
            is_visualised,
            is_streaming,
            is_static,
            is_snapping_desired,
            is_force_inside_triangle,
            is_dir_world_space,
        )
        self.logger.debug("RADAR - sensor created: " f"{self.name}")

    def _unpack_float(self, binary):
        # Convert the given binary string into a 1D array of floats.
        return np.frombuffer(binary, dtype=np.float32)

    def _decode_poll_data(self, binary):
        floats = self._unpack_float(binary)
        return floats.reshape((-1, 7))

    def remove(self):
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_radar()
        self.logger.debug("RADAR - sensor removed: " f"{self.name}")

    def poll(self):
        """
        Gets the most-recent raw readings for this RADAR sensor, if they exist.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary containing the 6D point cloud of raw RADAR data, where each entry is as follows:
            (range, doppler velocity, azimuth angle, elevation angle, radar cross section, signal to noise ratio).
        """
        # Send and receive a request for readings data from this sensor.
        binary = self.send_recv_ge("PollRadar", name=self.name)["data"]

        # If the data coming is empty, then it has the wrong type because of msgpack, create an empty bytes object
        if len(binary) == 0:
            binary = bytes()

        # Convert the binary string into an array of floats.
        radar_data = self._decode_poll_data(binary)
        self.logger.debug(
            "RADAR - sensor readings received from simulation: " f"{self.name}"
        )
        return radar_data

    def get_ppi(self):
        """
        Gets the latest RADAR PPI (plan position indicator) image from shared memory.

        Returns:
            The latest RADAR PPI (plan position indicator) image from shared memory.
        """
        self.send_recv_ge("GetPPIRadar", name=self.name)["data"]
        return np.frombuffer(self.shmem.read(self.shmem_size), dtype=np.uint8)

    def get_range_doppler(self):
        """
        Gets the latest RADAR Range-Doppler image from shared memory.

        Returns:
            The latest RADAR Range-Doppler image from shared memory.
        """
        self.send_recv_ge("GetRangeDopplerRadar", name=self.name)["data"]
        return np.frombuffer(self.shmem2.read(self.shmem_size), dtype=np.uint8)

    def stream_ppi(self):
        """
        Gets the latest RADAR PPI image from shared memory (which is being streamed directly).

        Returns:
            The latest RADAR PPI image from shared memory.
        """
        return np.frombuffer(self.shmem.read(self.shmem_size), dtype=np.uint8)

    def stream_range_doppler(self):
        """
        Gets the latest RADAR Range-Doppler image from shared memory (which is being streamed directly).

        Returns:
            The latest RADAR Range-Doppler image from shared memory.
        """
        return np.frombuffer(self.shmem2.read(self.shmem_size), dtype=np.uint8)

    def send_ad_hoc_poll_request(self) -> int:
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            A unique Id number for the ad-hoc request.
        """
        self.logger.debug("RADAR - ad-hoc polling request sent: " f"{self.name}")
        return int(self.send_recv_ge("SendAdHocRequestRadar", name=self.name)["data"])

    def is_ad_hoc_poll_request_ready(self, request_id: int) -> bool:
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug(
            "RADAR - ad-hoc polling request checked for completion: " f"{self.name}"
        )
        return self.send_recv_ge("IsAdHocPollRequestReadyRadar", requestId=request_id)[
            "data"
        ]

    def collect_ad_hoc_poll_request(self, request_id: int):
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            The readings data.
        """
        binary = self.send_recv_ge(
            "CollectAdHocPollRequestRadar", requestId=request_id
        )["data"]["radarData"]
        if len(binary) == 0:
            binary = bytes()
        radar_data = self._decode_poll_data(binary)

        self.logger.debug(
            "RADAR - ad-hoc polling request returned and processed: " f"{self.name}"
        )

        return radar_data

    def get_requested_update_time(self) -> float:
        """
        Gets the current 'requested update time' value for this sensor.

        Returns:
            (float): The requested update time.
        """
        return self.send_recv_ge("GetRadarRequestedUpdateTime", name=self.name)["data"]

    def get_update_priority(self) -> float:
        """
        Gets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, highest to lowest.

        Returns:
            The update priority value.
        """
        return self.send_recv_ge("GetRadarUpdatePriority", name=self.name)["data"]

    def get_position(self) -> Float3:
        """
        Gets the current world-space position of this sensor.

        Returns:
            The sensor position.
        """
        table = self.send_recv_ge("GetRadarSensorPosition", name=self.name)["data"]
        return (table["x"], table["y"], table["z"])

    def get_direction(self) -> Float3:
        """
        Gets the current direction vector of this sensor.

        Returns:
            The sensor direction.
        """
        table = self.send_recv_ge("GetRadarSensorDirection", name=self.name)["data"]
        return (table["x"], table["y"], table["z"])

    def get_max_pending_requests(self) -> int:
        """
        Gets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Returns:
            The max pending requests value.
        """
        return int(
            self.send_recv_ge("GetRadarMaxPendingGpuRequests", name=self.name)["data"]
        )

    def set_requested_update_time(self, requested_update_time: float):
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time: The new requested update time.
        """
        return self.send_ack_ge(
            "SetRadarRequestedUpdateTime",
            ack="CompletedSetRadarRequestedUpdateTime",
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
            "SetRadarUpdatePriority",
            ack="CompletedSetRadarUpdatePriority",
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
            "SetRadarMaxPendingGpuRequests",
            ack="CompletedSetRadarMaxPendingGpuRequests",
            name=self.name,
            maxPendingGpuRequests=max_pending_requests,
        )

    def _open_radar(
        self,
        name: str,
        vehicle: Vehicle | None,
        shmem_name: str | None,
        shmem2_name: str | None,
        shmem_size: int,
        requested_update_time: float,
        update_priority: float,
        pos: Float3,
        dir: Float3,
        up: Float3,
        range_bins: int,
        azimuth_bins: int,
        vel_bins: int,
        range_min: float,
        range_max: float,
        vel_min: float,
        vel_max: float,
        half_angle_deg: float,
        size: Int2,
        field_of_view_y: float,
        near_far_planes: Float2,
        range_roundness: float,
        range_cutoff_sensitivity: float,
        range_shape: float,
        range_focus: float,
        range_min_cutoff: float,
        range_direct_max_cutoff: float,
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
        data["shmemHandle2"] = shmem2_name
        data["shmemSize"] = shmem_size
        data["vid"] = 0
        if vehicle is not None:
            data["vid"] = vehicle.vid
        data["updateTime"] = requested_update_time
        data["priority"] = update_priority
        data["pos"] = pos
        data["dir"] = dir
        data["up"] = up
        data["range_bins"] = range_bins
        data["azimuth_bins"] = azimuth_bins
        data["vel_bins"] = vel_bins
        data["range_min"] = range_min
        data["range_max"] = range_max
        data["vel_min"] = vel_min
        data["vel_max"] = vel_max
        data["half_angle_deg"] = half_angle_deg
        data["size"] = size
        data["fovY"] = field_of_view_y
        data["near_far_planes"] = near_far_planes
        data["range_roundness"] = range_roundness
        data["range_cutoff_sensitivity"] = range_cutoff_sensitivity
        data["range_shape"] = range_shape
        data["range_focus"] = range_focus
        data["range_min_cutoff"] = range_min_cutoff
        data["range_direct_max_cutoff"] = range_direct_max_cutoff
        data["isVisualised"] = is_visualised
        data["isStreaming"] = is_streaming
        data["isStatic"] = is_static
        data["isSnappingDesired"] = is_snapping_desired
        data["isForceInsideTriangle"] = is_force_inside_triangle
        data["isDirWorldSpace"] = is_dir_world_space

        self.send_ack_ge(type="OpenRadar", ack="OpenedRadar", **data)
        self.logger.info(f'Opened RADAR sensor: "{name}"')

    def _close_radar(self) -> None:
        self.send_ack_ge(type="CloseRadar", ack="ClosedRadar", name=self.name)
        self.logger.info(f'Closed RADAR sensor: "{self.name}"')

    def plot_data(
        self,
        readings_data,
        resolution,
        field_of_view_y,
        range_min,
        range_max,
        range_bins: int = 200,
        azimuth_bins: int = 200,
    ):
        """
        Plot the RADAR readings data. The data plots are: B-Scope, PPI (Plan Position Indicator), RCS (Radar Cross Section), and SNR (Signal-to-Noise Ratio).
        The data is used to populate bins, where each bin represents one pixel on the images, and contains a weighted average of the data at
        that location.
        If data exists outside of the given distance/angle ranges, it will be snapped to the nearest bin, so this should be avoided by providing
        accurate limits for these.

        Args:
            readings_data: The readings data structure obtained from polling the RADAR sensor.
            resolution: (X, Y) The resolution of the sensor (the size of the depth buffer image in the distance measurement computation).
            field_of_view_y: The vertical field of view of the RADAR, in degrees.
            range_min: The minimum range of the sensor, in metres.
            range_max: The maximum range of the sensor, in metres.
            range_bins: The number of bins to use for the range dimension, in the data plots.
            azimuth_bins: The number of bins to use for the azimuth dimension, in the data plots.
        """

        # Iterate over all RADAR readings, and populate the data into bins.
        rows = range_bins + 1
        cols = azimuth_bins + 1
        velocity_bins = np.zeros(
            [rows, cols]
        )  # Stores the sum of the Doppler velocities for each bin.
        RCS_bins = np.zeros([rows, cols])  # Stores the sum of RCS for each bin.
        SNR_bins = np.zeros([rows, cols])  # Stores the sum of SNR for each bin.
        tally_bins = np.zeros(
            [rows, cols]
        )  # Counts the number of entries for each bin.
        fov_azimuth = (resolution[0] / float(resolution[1])) * field_of_view_y
        fov_rad = np.deg2rad(fov_azimuth)
        max_az_rad = fov_rad / 2
        min_az_rad = -max_az_rad
        range_size = range_max - range_min
        for i in range(len(readings_data)):

            # Find the appropriate 2D bin index (distance, azimuth) for this reading.
            a = int(
                math.floor(
                    ((readings_data[i][2] - min_az_rad) / fov_rad) * azimuth_bins
                )
            )
            d = int(
                math.floor(
                    ((readings_data[i][0] - range_min) / range_size) * range_bins
                )
            )
            # Safety: if any data is outside the range of the bins, snap it to the nearest edge bin.
            d = max(0, min(range_bins, d))

            # For the appropriate bin, increment its number of entries for later averaging purposes.
            tally_bins[d, a] = tally_bins[d, a] + 1

            # Add the weighted doppler velocity, weighted RCS, and weighted SNR values to the appropriate bins.
            weight = readings_data[i][6]
            velocity_bins[d, a] = velocity_bins[d, a] + readings_data[i][1] * weight
            # We convert to dB scale.
            RCS_bins[d, a] = (
                RCS_bins[d, a] + (10.0 * math.log10(readings_data[i][4])) * weight
            )
            # We convert to dB scale.
            SNR_bins[d, a] = (
                SNR_bins[d, a] + (10.0 * math.log10(readings_data[i][5])) * weight
            )

        # Iterate over all bins and perform the averaging.
        for r in range(rows):
            for c in range(cols):
                if tally_bins[r, c] > 0.0:
                    tally_reciprocal = 1.0 / tally_bins[r, c]
                    velocity_bins[r, c] = velocity_bins[r, c] * tally_reciprocal
                    RCS_bins[r, c] = RCS_bins[r, c] * tally_reciprocal
                    SNR_bins[r, c] = SNR_bins[r, c] * tally_reciprocal

        # Create the B-Scope Plot.
        fig, ax = plt.subplots(2, 2, figsize=(15, 15))
        half_fov_azimuth = fov_azimuth / 2
        im = ax[0, 0].imshow(
            velocity_bins,
            aspect="auto",
            origin="lower",
            extent=(-half_fov_azimuth, half_fov_azimuth, range_min, range_max),
        )
        ax[0, 0].set_title("B-Scope")
        ax[0, 0].set_xlabel("Azimuth (degrees)")
        ax[0, 0].set_ylabel("Range (m)")
        ax[0, 0].set_aspect(0.75)
        fig.colorbar(im, ax=ax[0, 0])

        # Create a grid of vertices that approximate the bounds of each radar cell (in polar coordinates), then convert from polar to Cartesian coordinates.
        r, az = np.mgrid[
            0.0 : range_max : (range_max / (range_bins + 1)),
            max_az_rad : min_az_rad : -((max_az_rad - min_az_rad) / (azimuth_bins + 1)),
        ]
        az_plus_half_pi = az + (np.pi / 2)
        grid_x = r * np.cos(az_plus_half_pi)
        grid_y = r * np.sin(az_plus_half_pi)

        # Create the PPI (Plan Position Indicator) Plot.
        mesh = ax[1, 0].pcolormesh(grid_x, grid_y, velocity_bins)
        ax[1, 0].set_title("PPI (Plan Position Indicator)")
        ax[1, 0].set_xlabel("Cross-range (m)")
        ax[1, 0].set_ylabel("Down-range (m)")
        ax[1, 0].set_aspect("equal")
        fig.colorbar(mesh, ax=ax[1, 0])

        # Create the SNR (Signal-to-Noise Ratio) Plot.
        mesh = ax[0, 1].pcolormesh(grid_x, grid_y, SNR_bins)
        ax[0, 1].set_title("SNR (Signal-to-Noise Ratio) dB")
        ax[0, 1].set_xlabel("Cross-range (m)")
        ax[0, 1].set_ylabel("Down-range (m)")
        ax[0, 1].set_aspect("equal")
        fig.colorbar(mesh, ax=ax[0, 1])

        # Create the RCS (Radar Cross Section) Plot.
        mesh = ax[1, 1].pcolormesh(grid_x, grid_y, RCS_bins)
        ax[1, 1].set_title("RCS (Radar Cross Section) dB")
        ax[1, 1].set_xlabel("Cross-range (m)")
        ax[1, 1].set_ylabel("Down-range (m)")
        ax[1, 1].set_aspect("equal")
        fig.colorbar(mesh, ax=ax[1, 1])
        plt.show()

    def plot_velocity_data(
        self,
        velocity_data,
        resolution,
        field_of_view_y,
        range_min: float = 0.0,
        range_max: float = 100.0,
        range_bins: int = 200,
        azimuth_bins: int = 200,
    ):
        """
        Plot the RADAR Doppler velocities.

        Args:
            velocity_data: The 2D velocity array obtained from the RADAR sensor.
            resolution: (X, Y) The resolution of the sensor (the size of the depth buffer image in the distance measurement computation).
            field_of_view_y: The vertical field of view of the RADAR, in degrees.
            range_min: The minimum range of the sensor, in metres.
            range_max: The maximum range of the sensor, in metres.
            range_bins: The number of bins to use for the range dimension, in the data plots.
            azimuth_bins: The number of bins to use for the azimuth dimension, in the data plots.
        """

        fov_azimuth = (resolution[0] / float(resolution[1])) * field_of_view_y
        half_fov_azimuth = fov_azimuth * 0.5
        fov_rad = np.deg2rad(fov_azimuth)
        max_az_rad = fov_rad / 2
        min_az_rad = -max_az_rad

        # Create the B-Scope Plot.
        fig, ax = plt.subplots(2, 2, figsize=(15, 15))
        im = ax[0, 0].imshow(
            velocity_data,
            aspect="auto",
            origin="lower",
            extent=(-half_fov_azimuth, half_fov_azimuth, range_min, range_max),
        )
        ax[0, 0].set_title("B-Scope")
        ax[0, 0].set_xlabel("Azimuth (degrees)")
        ax[0, 0].set_ylabel("Range (m)")
        ax[0, 0].set_aspect(0.75)
        ax[0, 0].grid(False)
        fig.colorbar(im, ax=ax[0, 0])

        # Create a grid of vertices that approximate the bounds of each radar cell (in polar coordinates), then convert from polar to Cartesian coordinates.
        r, az = np.mgrid[
            0.0 : range_max : (range_max / (range_bins + 1)),
            max_az_rad : min_az_rad : -((max_az_rad - min_az_rad) / (azimuth_bins + 1)),
        ]
        az_plus_half_pi = az + (np.pi / 2)
        grid_x = r * np.cos(az_plus_half_pi)
        grid_y = r * np.sin(az_plus_half_pi)

        # Create the PPI (Plan Position Indicator) Plot.
        mesh = ax[1, 0].pcolormesh(grid_x, grid_y, velocity_data)
        ax[1, 0].set_title("PPI (Plan Position Indicator)")
        ax[1, 0].set_xlabel("Cross-range (m)")
        ax[1, 0].set_ylabel("Down-range (m)")
        ax[1, 0].set_aspect("equal")
        fig.colorbar(mesh, ax=ax[1, 0])
        plt.show()
