from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Dict, List

import numpy as np
from PIL import Image

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID, BNGError, BNGValueError
from beamngpy.sensors.shmem import BNGSharedMemory
from beamngpy.types import Float2, Float3, Int2, Int3, StrDict

from . import utils

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle


class Camera(CommBase):
    """
    An interactive, automated camera sensor, which can produce regular colour images, depth images, or annotation images.
    This sensor can be attached to a vehicle, or can be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
    A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
    will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.

    Args:
        name: A unique name for this camera sensor.
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this sensor should be attached, if any.
        requested_update_time: The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
        update_priority: The priority which the sensor should ask for new readings. lowest -> 0, highest -> 1.
        pos: (X, Y, Z) Coordinate triplet specifying the position of the sensor, in world space.
        dir: (X, Y, Z) Coordinate triplet specifying the forward direction of the sensor.
        up: (X, Y, Z) Coordinate triplet specifying the up direction of the sensor.
        resolution: (X, Y) The resolution of the sensor images.
        field_of_view_y: The sensor vertical field of view parameters.
        near_far_planes: (X, Y) The sensor near and far plane distances.
        is_using_shared_memory: A flag which indicates if we should use shared memory to send/recieve the sensor readings data.
        is_render_colours: A flag which indicates if this sensor should render colour data.
        is_render_annotations: A flag which indicates if this sensor should render semantic annotation data.
        is_render_instance: A flag which indicates if this sensor should render instance annotation data.
        is_render_depth: A flag which indicates if this sensor should render depth data.
        is_depth_inverted: A flag which indicates if the depth values should be shown white->black or black->white, as distance increases.
        is_visualised: A flag which indicates if this camera sensor should appear visualised or not.
        is_streaming: Whether or not to stream the data directly to shared memory (no poll required, for efficiency - BeamNGpy won't block.)
        is_static: A flag which indicates whether this sensor should be static (fixed position), or attached to a vehicle.
        is_snapping_desired: A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle (not used for static sensors).
        is_force_inside_triangle: A flag which indicates if the sensor should be forced inside the nearest vehicle triangle (not used for static sensors).
        postprocess_depth: If True, the raw depth data will be postprocessed to better represent values with middle intensity. Defaults to False,
                           as the postprocessing is computationally intensive.
        is_dir_world_space: Flag which indicates if the direction is provided in world-space coordinates (True), or the default vehicle space (False).
    """

    @staticmethod
    def extract_bounding_boxes(
        semantic_image: Image.Image, instance_image: Image.Image, classes: StrDict
    ) -> List[StrDict]:
        """
        Analyzes the given semantic annotation and instance annotation images for its object bounding boxes. The identified objects are returned as
        a list of dictionaries containing their bounding box corners, class of object according to the corresponding colour in the semantic
        annotations and the given class mapping, and the colour of the object in the instance annotation.

        Args:
            semantic_image: The image containing semantic annotation information.
            instance_image: The image containing instance annotation information.
            classes: A mapping of colours to their class names to identify object types based on the semantic annotation information. The keys in
                    this dictionary are the respective colours expressed as a 24-bit integer, i.e. [r * 256^2 + g * 256 + b].

        Returns:
            A list of bounding boxes specified as dictionaries. Example: 'bbox': [min_x, min_y, max_x, max_y], 'color': [233, 11, 15], 'class': ['CAR'],
            where min_x, min_y, max_x, max_y mark the corners of the bounding box, colour contains the RGB colour of the object in the instance
            annotations, and class the object type identified through the given class mapping.
        """
        return utils.extract_bounding_boxes(semantic_image, instance_image, classes)

    @staticmethod
    def draw_bounding_boxes(
        bounding_boxes: List[StrDict],
        colour: Image.Image,
        width: int = 3,
        font: str = "arial.ttf",
        font_size: int = 14,
    ) -> Image.Image:
        """
        Draws the given list of bounding boxes onto the given image. The boxes are drawn with the given width of outlines in pixels and the given font
        and size configuration. NOTE: The given image is not directly modified and the boxes are drawn onto a copy.

        Args:
            bounding_boxes: List of bounding boxes to draw.
            colour: The image to draw the bounding boxes on.
            width: The width of bounding box outlines in pixels.
            font: A string specifying the font which bounding box labels will have.
            font_size: The font size used when drawing labels.

        Returns:
            An :class:`Image` that is a copy of the given image with bounding boxes drawn onto it.
        """
        return utils.draw_bounding_boxes(bounding_boxes, colour, width, font, font_size)

    @staticmethod
    def export_bounding_boxes_xml(
        bounding_boxes: List[StrDict],
        folder: str | None = None,
        filename: str | None = None,
        path: str | None = None,
        database: str | None = None,
        size: Int3 | None = None,
    ) -> str:
        """
        Exports the given list of bounding boxes to the Pascal-VOC XML standard. Additional properties to this function correspond to tags in the
        Pascal-VOC standard.

        Args:
            bounding_boxes: The list of bounding boxes to export.
            folder: Contents of the 'folder' tag.
            filename: Contents of the 'filename' tag.
            path: Contents of the 'path' tag.
            database: Contents of the 'database' tag.
            size: Contents of the 'size tag. It's expected to be a tuple of the image width, height, and depth.

        Returns:
            XML string encoding of the given list of bounding boxes according to Pascal-VOC.
        """
        return utils.export_bounding_boxes_xml(
            bounding_boxes, folder, filename, path, database, size
        )

    def __init__(
        self,
        name: str,
        bng: BeamNGpy,
        vehicle: Vehicle | None = None,
        requested_update_time: float = 0.1,
        update_priority: float = 0.0,
        pos: Float3 = (0, 0, 3),
        dir: Float3 = (0, -1, 0),
        up: Float3 = (0, 0, 1),
        resolution: Int2 = (512, 512),
        field_of_view_y: float = 70,
        near_far_planes: Float2 = (0.05, 100.0),
        is_using_shared_memory: bool = False,
        is_render_colours: bool = True,
        is_render_annotations: bool = True,
        is_render_instance: bool = False,
        is_render_depth: bool = True,
        is_depth_inverted: bool = False,
        is_visualised: bool = False,
        is_streaming: bool = False,
        is_static: bool = False,
        is_snapping_desired: bool = False,
        is_force_inside_triangle: bool = False,
        postprocess_depth: bool = False,
        is_dir_world_space: bool = False,
    ):
        super().__init__(bng, vehicle)
        self.logger = getLogger(f"{LOGGER_ID}.Camera")
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.name = name
        self.resolution = resolution
        self.near_far_planes = near_far_planes
        self.is_static = is_static
        self.is_depth_inverted = is_depth_inverted
        self.is_render_colours = is_render_colours
        self.is_render_annotations = is_render_annotations
        self.is_render_instance = is_render_instance
        self.is_render_depth = is_render_depth
        self.is_streaming = is_streaming
        self.postprocess_depth = postprocess_depth

        # Set up the shared memory for this sensor, if requested.
        self.is_using_shared_memory = is_using_shared_memory
        self.colour_shmem = None
        self.annotation_shmem = None
        self.instance_shmem = None
        self.depth_shmem = None
        self.shmem_size = -1
        if is_using_shared_memory:
            self.logger.debug("Camera - Initializing shared memory.")
            self.shmem_size = resolution[0] * resolution[1] * 4
            if is_render_colours:
                self.colour_shmem = BNGSharedMemory(self.shmem_size)
                self.logger.debug(
                    "Camera - Bound shared memory for colour: "
                    f"{self.colour_shmem.name}"
                )

            if is_render_annotations:
                self.annotation_shmem = BNGSharedMemory(self.shmem_size)
                self.logger.debug(
                    "Camera - Bound shared memory for semantic annotations: "
                    f"{self.annotation_shmem.name}"
                )

            if is_render_instance:
                self.instance_shmem = BNGSharedMemory(self.shmem_size)
                self.logger.debug(
                    "Camera - Bound shared memory for instance annotations: "
                    f"{self.instance_shmem.name}"
                )

            if is_render_depth:
                self.depth_shmem = BNGSharedMemory(self.shmem_size)
                self.logger.debug(
                    "Camera - Bound shared memory for depth: "
                    f"{self.depth_shmem.name}"
                )

        # Create and initialise the camera in the simulation.
        colour_shmem_name = self.colour_shmem.name if self.colour_shmem else None
        annotation_shmem_name = (
            self.annotation_shmem.name if self.annotation_shmem else None
        )
        depth_shmem_name = self.depth_shmem.name if self.depth_shmem else None
        self._open_camera(
            name,
            vehicle,
            requested_update_time,
            update_priority,
            self.resolution,
            field_of_view_y,
            near_far_planes,
            pos,
            dir,
            up,
            is_using_shared_memory,
            colour_shmem_name,
            self.shmem_size,
            annotation_shmem_name,
            self.shmem_size,
            depth_shmem_name,
            self.shmem_size,
            is_render_colours,
            is_render_annotations,
            is_render_instance,
            is_render_depth,
            is_visualised,
            is_streaming,
            is_static,
            is_snapping_desired,
            is_force_inside_triangle,
            is_dir_world_space,
        )
        self.logger.debug("Camera - sensor created: " f"{self.name}")

    def _convert_to_image(
        self, raw_data: bytes | str | None, width: int, height: int
    ) -> Image.Image | None:
        """
        Converts raw image data from the simulator into image format.

        Args:
            raw_data: The 1D buffer to be processed.
            width: The width of the image to be rendered.
            height: The height of the image to be rendered.

        Returns:
            The processed image.
        """
        if raw_data is None or len(raw_data) == 0:
            return None

        data = raw_data.encode() if isinstance(raw_data, str) else raw_data

        # Re-shape the array, based on the number of channels present in the data.
        decoded = np.frombuffer(data, dtype=np.uint8)
        decoded = decoded.reshape(height, width, 4).copy()

        # Convert to image format.
        b = Image.fromarray(decoded)

        return b

    def depth_buffer_processing(self, raw_depth_values: np.ndarray) -> np.ndarray:
        """
        Converts raw depth buffer data to visually-clear intensity values in the range [0, 255].
        We process the data so that small changes in distance are better shown, rather than just using linear interpolation.

        Args:
            raw_depth_values: The raw 1D buffer of depth values.

        Returns:
            The processed intensity values in the range [0, 255].
        """
        # Sort the depth values, and cache the sorting map.
        sort_index = np.argsort(raw_depth_values)
        s_data = raw_depth_values[sort_index]

        # Compute an array of unique depth values, sensitive to some epsilon.
        eps = 0.01
        rounded_depth = (s_data * (1 // eps)).astype(np.int32)
        _, indices = np.unique(rounded_depth, return_index=True)
        unique = s_data[indices]

        # Distribute (mark) the individual intensity values throughout the sorted unique distance array.
        intensity_marks = np.empty((256,))
        intensity_marks[0] = 0
        intensity_marks[-1] = 1e12
        quantiles = np.arange(254) * (len(unique) / 255.0)
        quantiles = quantiles.astype(np.int32)
        intensity_marks[1:255] = unique[quantiles]

        # In the sorted depth values array, convert the depth value array into intensity values.
        depth_intensity_sorted = np.zeros_like(s_data)
        last_idx = 0
        for i in range(1, 256):
            idx = (
                np.searchsorted(s_data[last_idx:], intensity_marks[i], "left")
                + 1
                + last_idx
            )
            depth_intensity_sorted[last_idx:idx] = i - 1
            last_idx = idx

        # Re-map the depth values back to their original order.
        depth_intensity = np.empty_like(s_data)
        depth_intensity[sort_index] = depth_intensity_sorted

        return depth_intensity

    def _binary_to_image(
        self, binary: StrDict, full_poll_request: bool = False
    ) -> Dict[str, Image.Image | None]:
        """
        Converts the binary string data from the simulator, which contains the data buffers for colour, annotations, and depth, into images.

        Args:
            binary: The raw readings data, as a binary string.
        Returns:
            A dictionary containing the processed images.
        """
        width = int(self.resolution[0])
        height = int(self.resolution[1])

        processed_readings: Dict[str, Image.Image | None] = dict()
        if self.is_render_colours:
            processed_readings["colour"] = self._convert_to_image(
                binary.get("colour"), width, height
            )

        if self.is_render_annotations:
            processed_readings["annotation"] = self._convert_to_image(
                binary.get("annotation"), width, height
            )

        if self.is_render_instance:
            processed_readings["instance"] = self._convert_to_image(
                binary.get("instance"), width, height
            )

        if self.is_render_depth:
            if binary.get("depth") is None or len(binary["depth"]) == 0:
                processed_readings["depth"] = None
            else:
                depth = np.frombuffer(binary["depth"], dtype=np.float32)
                if full_poll_request:  # transform the (NEAR, FAR) range to (0.0, 1.0)
                    depth = (depth - self.near_far_planes[0]) / (
                        self.near_far_planes[1] - self.near_far_planes[0]
                    )

                if self.postprocess_depth:
                    depth = self.depth_buffer_processing(depth)
                else:  # scale from (0.0, 1.0) to (0.0, 255.0)
                    depth = np.clip(depth * 255.0, 0.0, 255.0)
                reshaped_data = np.array(depth.reshape(height, width), dtype=np.uint8)
                if self.is_depth_inverted:
                    reshaped_data = 255 - reshaped_data
                image = Image.fromarray(reshaped_data)
                processed_readings["depth"] = image

        return processed_readings

    def remove(self) -> None:
        """
        Removes this sensor from the simulation.
        """
        # Remove any shared memory binding which this sensor is using.
        if self.is_using_shared_memory:
            if self.colour_shmem:
                self.logger.debug(
                    "Camera - Unbinding shared memory for colour: "
                    f"{self.colour_shmem.name}"
                )
                self.colour_shmem.try_close()

            if self.annotation_shmem:
                self.logger.debug(
                    "Camera - Unbinding shared memory for semantic annotations: "
                    f"{self.annotation_shmem.name}"
                )
                self.annotation_shmem.try_close()

            if self.instance_shmem:
                self.logger.debug(
                    "Camera - Unbinding shared memory for instance annotations: "
                    f"{self.instance_shmem.name}"
                )
                self.instance_shmem.try_close()

            if self.depth_shmem:
                self.logger.debug(
                    "Camera - Unbinding shared memory for depth: "
                    f"{self.depth_shmem.name}"
                )
                self.depth_shmem.try_close()

        # Remove this sensor from the simulation.
        self._close_camera()
        self.logger.debug("Camera - sensor removed: " f"{self.name}")

    def poll_raw(self) -> Dict[str, bytes | None]:
        """
        Gets the most-recent readings for this sensor as unprocessed bytes.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary with values being the unprocessed bytes representing the RGBA data from the sensors and
            the following keys

            * ``colour``: The colour data.
            * ``annotation``: The semantic annotation data.
            * ``depth``: The depth camera data.
        """

        # Send and receive a request for readings data from this sensor.
        raw_readings = self.send_recv_ge(
            "PollCamera",
            name=self.name,
            isUsingSharedMemory=self.is_using_shared_memory,
        )["data"]

        if self.is_using_shared_memory:
            if self.colour_shmem:
                if "colour" in raw_readings.keys():
                    raw_readings["colour"] = self.colour_shmem.read(self.shmem_size)
                else:
                    self.logger.error(
                        "Camera - Colour buffer failed to render. Check that you are not running on low settings."
                    )
            if self.annotation_shmem:
                if "annotation" in raw_readings.keys():
                    raw_readings["annotation"] = self.annotation_shmem.read(
                        self.shmem_size
                    )
                else:
                    self.logger.error(
                        "Camera - Annotation buffer failed to render. Check that you are not running on low settings."
                    )
            if self.depth_shmem:
                if "depth" in raw_readings.keys():
                    raw_readings["depth"] = self.depth_shmem.read(self.shmem_size)
                else:
                    self.logger.error(
                        "Camera - Depth buffer failed to render. Check that you are not running on low settings."
                    )
            self.logger.debug(
                "Camera - sensor readings read from shared memory and processed: "
                f"{self.name}"
            )

        return raw_readings

    def poll(self):
        """
        Gets the most-recent readings for this sensor as processed images.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary with the values as processed images and the following keys

            * ``colour``: The colour data.
            * ``annotation``: The semantic annotation data.
            * ``depth``: The depth camera data.
        """
        raw_readings = self.poll_raw()
        images = self._binary_to_image(raw_readings)
        return images

    def stream_raw(self) -> Dict[str, bytes]:
        """
        Gets the most-recent readings for this sensor as unprocessed bytes without sending a request to the simulator.
        Can only be called in the case that the Camera sensor was constructed with ``is_streaming=True``.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary with values being the unprocessed bytes representing the RGBA data from the sensors and the
            following keys

            * ``colour``: The colour data.
            * ``annotation``: The semantic annotation data.
            * ``depth``: The depth camera data.
        """
        if not self.is_streaming:
            raise BNGError(
                "This camera sensor was not created with `is_streaming=True`. Stream not available."
            )

        raw_readings = {}
        if self.colour_shmem:
            raw_readings["colour"] = self.colour_shmem.read(self.shmem_size)
        if self.annotation_shmem:
            raw_readings["annotation"] = self.annotation_shmem.read(self.shmem_size)
        if self.depth_shmem:
            raw_readings["depth"] = self.depth_shmem.read(self.shmem_size)

        return raw_readings

    def stream(self) -> Dict[str, Image.Image | None]:
        """
        Gets the most-recent readings for this sensor as processed images without sending a request to the simulator.
        Can only be called in the case that the Camera sensor was constructed with ``is_streaming=True``.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary with the values as processed images and the following keys

            * ``colour``: The colour data.
            * ``annotation``: The semantic annotation data.
            * ``depth``: The depth camera data.
        """

        if not self.is_streaming:
            raise BNGError(
                "This camera sensor was not created with `is_streaming=True`. Stream not available."
            )

        raw_readings = self.stream_raw()
        images = self._binary_to_image(raw_readings)
        self.logger.debug(
            "Camera - raw sensor readings converted to image format: " f"{self.name}"
        )
        return images

    def poll_shmem_colour(self):
        self.send_recv_ge(
            "PollCamera",
            name=self.name,
            isUsingSharedMemory=self.is_using_shared_memory,
        )
        width = self.resolution[0]
        height = self.resolution[1]
        img = np.frombuffer(self.colour_shmem.read(width * height * 4), dtype=np.uint8)
        return [img, width, height]

    def poll_shmem_annotation(self):
        self.send_recv_ge(
            "PollCamera",
            name=self.name,
            isUsingSharedMemory=self.is_using_shared_memory,
        )
        width = self.resolution[0]
        height = self.resolution[1]
        img = np.frombuffer(
            self.annotation_shmem.read(width * height * 4), dtype=np.uint8
        )
        return [img, width, height]

    def poll_shmem_depth(self):
        self.send_recv_ge(
            "PollCamera",
            name=self.name,
            isUsingSharedMemory=self.is_using_shared_memory,
        )
        width = self.resolution[0]
        height = self.resolution[1]
        img = np.frombuffer(self.depth_shmem.read(width * height * 4), dtype=np.float32)
        return [img, width, height]

    def stream_colour(self, size):
        return np.frombuffer(self.colour_shmem.read(size), dtype=np.uint8)

    def stream_annotation(self, size):
        return np.frombuffer(self.annotation_shmem.read(size), dtype=np.uint8)

    def stream_depth(self, size):
        return np.frombuffer(self.depth_shmem.read(size), dtype=np.float32)

    def send_ad_hoc_poll_request(self) -> int:
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            A unique Id number for the ad-hoc request.
        """
        self.logger.debug("Camera - ad-hoc polling request sent: " f"{self.name}")
        return int(self.send_recv_ge("SendAdHocRequestCamera", name=self.name)["data"])

    def is_ad_hoc_poll_request_ready(self, request_id: int) -> bool:
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug(
            "Camera - ad-hoc polling request checked for completion: " f"{self.name}"
        )
        return self.send_recv_ge("IsAdHocPollRequestReadyCamera", requestId=request_id)[
            "data"
        ]

    def collect_ad_hoc_poll_request(self, request_id: int) -> StrDict:
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            The readings data.
        """
        # Obtain the raw readings (as binary strings) from the simulator, for this ad-hoc polling request.
        raw_readings = self.send_recv_ge(
            "CollectAdHocPollRequestCamera", requestId=request_id
        )["data"]

        # Format the binary string data from the simulator.
        return self._binary_to_image(raw_readings)

    # TODO: Should be removed when GE-2170 is complete.
    def get_full_poll_request(self) -> StrDict:
        """
        Gets a full camera request (semantic annotation and instance annotation data included).
        NOTE: this function blocks the simulation until the data request is completed.

        Returns:
            The camera data, as images
        """
        # Obtain the raw readings (as binary strings) from the simulator, for this ad-hoc polling request.
        raw_readings = self.send_recv_ge("GetFullCameraRequest", name=self.name)
        if "data" not in raw_readings:
            raise BNGValueError(f"Camera sensor {self.name} not found.")
        raw_readings = raw_readings["data"]
        raw_readings = self._binary_to_image(raw_readings, full_poll_request=True)

        data = dict(type="data")
        data["colour"] = raw_readings["colour"]
        data["annotation"] = raw_readings["annotation"]
        data["instance"] = raw_readings["instance"]
        data["depth"] = raw_readings["depth"]

        # Format the binary string data from the simulator.
        return data

    def world_point_to_pixel(self, point: Float3) -> Int2:
        """
        Converts a 3D point in world space to the 2D pixel coordinate at which it is represented on this camera.
        NOTE: The pixel does not have to actually be visible on the camera image itself in order to retrieve a value; it can be obscured by geometry
        which is closer, or it can be run without respect to the near and far plane values of the camera.

        Args:
            point: The given 3D point, in world space coordinates.

        Returns:
            The 2D pixel value which represents the given 3D point, on this camera.
        """
        pixel_data = self.send_recv_ge(
            "CameraWorldPointToPixel",
            name=self.name,
            pointX=point[0],
            pointY=point[1],
            pointZ=point[2],
        )["data"]
        return (int(pixel_data["x"]), int(pixel_data["y"]))

    def get_position(self) -> Float3:
        """
        Gets the current world-space position of this sensor.

        Returns:
            The sensor position.
        """
        table = self.send_recv_ge("GetCameraSensorPosition", name=self.name)["data"]
        return (table["x"], table["y"], table["z"])

    def get_direction(self) -> Float3:
        """
        Gets the current forward direction vector of this sensor.

        Returns:
            The sensor direction.
        """
        table = self.send_recv_ge("GetCameraSensorDirection", name=self.name)["data"]
        return (table["x"], table["y"], table["z"])

    def get_requested_update_time(self) -> float:
        """
        Gets the current 'requested update time' value for this sensor.

        Returns:
            The requested update time.
        """
        return self.send_recv_ge("GetCameraRequestedUpdateTime", name=self.name)["data"]

    def get_update_priority(self) -> float:
        """
        Gets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, highest to lowest.

        Returns:
            The update priority value.
        """
        return self.send_recv_ge("GetCameraUpdatePriority", name=self.name)["data"]

    def get_max_pending_requests(self) -> int:
        """
        Gets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Returns:
            The max pending requests value.
        """
        return int(
            self.send_recv_ge("GetCameraMaxPendingGpuRequests", name=self.name)["data"]
        )

    def set_position(self, pos: Float3) -> None:
        """
        Sets the current world-space position for this sensor.

        Args:
            pos: The new position.
        """
        return self.send_ack_ge(
            "SetCameraSensorPosition",
            ack="CompletedSetCameraSensorPosition",
            name=self.name,
            posX=pos[0],
            posY=pos[1],
            posZ=pos[2],
        )

    def set_direction(self, dir: Float3) -> None:
        """
        Sets the current forward direction vector of this sensor.

        Args:
            dir: The new forward direction vector.
        """
        self.send_ack_ge(
            "SetCameraSensorDirection",
            ack="CompletedSetCameraSensorDirection",
            name=self.name,
            dirX=dir[0],
            dirY=dir[1],
            dirZ=dir[2],
        )

    def set_up(self, up: Float3) -> None:
        """
        Sets the current up vector of this sensor.

        Args:
            pos: The new up vector.
        """
        self.send_ack_ge(
            "SetCameraSensorUp",
            ack="CompletedSetCameraSensorUp",
            name=self.name,
            upX=up[0],
            upY=up[1],
            upZ=up[2],
        )

    def set_requested_update_time(self, requested_update_time: float) -> None:
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time: The new requested update time.
        """
        return self.send_ack_ge(
            "SetCameraRequestedUpdateTime",
            ack="CompletedSetCameraRequestedUpdateTime",
            name=self.name,
            updateTime=requested_update_time,
        )

    def set_update_priority(self, update_priority: float) -> None:
        """
        Sets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, , highest to lowest.

        Args:
            update_priority: The new update priority value.
        """
        return self.send_ack_ge(
            "SetCameraUpdatePriority",
            ack="CompletedSetCameraUpdatePriority",
            name=self.name,
            updatePriority=update_priority,
        )

    def set_max_pending_requests(self, max_pending_requests: int) -> None:
        """
        Sets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Args:
            max_pending_requests: The new max pending requests value.
        """
        return self.send_ack_ge(
            "SetCameraMaxPendingGpuRequests",
            ack="CompletedSetCameraMaxPendingGpuRequests",
            name=self.name,
            maxPendingGpuRequests=max_pending_requests,
        )

    def _open_camera(
        self,
        name: str,
        vehicle: Vehicle | None,
        requested_update_time: float,
        update_priority: float,
        size: Int2,
        field_of_view_y: float,
        near_far_planes: Float2,
        pos: Float3,
        dir: Float3,
        up: Float3,
        is_using_shared_memory: bool,
        colour_shmem_handle: str | None,
        colour_shmem_size: int,
        annotation_shmem_handle: str | None,
        annotation_shmem_size: int,
        depth_shmem_handle: str | None,
        depth_shmem_size: int,
        is_render_colours: bool,
        is_render_annotations: bool,
        is_render_instance: bool,
        is_render_depth: bool,
        is_visualised: bool,
        is_streaming: bool,
        is_static: bool,
        is_snapping_desired: bool,
        is_force_inside_triangle: bool,
        is_dir_world_space: bool,
    ) -> None:
        data: StrDict = dict()
        data["vid"] = 0
        if vehicle is not None:
            data["vid"] = vehicle.vid
        data["name"] = name
        data["updateTime"] = requested_update_time
        data["priority"] = update_priority
        data["size"] = size
        data["fovY"] = field_of_view_y
        data["nearFarPlanes"] = near_far_planes
        data["pos"] = pos
        data["dir"] = dir
        data["up"] = up
        data["useSharedMemory"] = is_using_shared_memory
        data["colourShmemName"] = colour_shmem_handle
        data["colourShmemSize"] = colour_shmem_size
        data["annotationShmemName"] = annotation_shmem_handle
        data["annotationShmemSize"] = annotation_shmem_size
        data["depthShmemName"] = depth_shmem_handle
        data["depthShmemSize"] = depth_shmem_size
        data["renderColours"] = is_render_colours
        data["renderAnnotations"] = is_render_annotations
        data["renderInstance"] = is_render_instance
        data["renderDepth"] = is_render_depth
        data["isVisualised"] = is_visualised
        data["isStreaming"] = is_streaming
        data["isStatic"] = is_static
        data["isSnappingDesired"] = is_snapping_desired
        data["isForceInsideTriangle"] = is_force_inside_triangle
        data["isDirWorldSpace"] = is_dir_world_space
        self.send_ack_ge(type="OpenCamera", ack="OpenedCamera", **data)
        self.logger.info(f'Opened Camera: "{name}"')

    def _close_camera(self) -> None:
        self.send_ack_ge(type="CloseCamera", ack="ClosedCamera", name=self.name)
        self.logger.info(f'Closed Camera: "{self.name}"')
