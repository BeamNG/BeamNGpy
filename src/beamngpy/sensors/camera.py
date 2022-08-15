"""
An interactive, automated camera sensor, which can produce regular colour images, depth images, or annotation images.
This sensor can be attached to a vehicle, or can be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.
"""

import math
import mmap
import numbers
import os
import struct
from logging import DEBUG, getLogger
from xml.dom import minidom
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement

import numpy as np
from beamngpy.beamngcommon import LOGGER_ID, BNGValueError
from PIL import Image, ImageDraw, ImageFont, ImageOps

from .sensor import PollingSensor

NEAR = 0.01
FAR = 1000


class Camera(PollingSensor):
    """
    A camera sensor provides several types of image data from a user-defined
    perspective relative to the vehicle. It can provide the following types of
    data:

    * Colour images
    * Pixel-wise depth
    * Pixel-wise object annotation
    * Pixel-wise instance annotation to separate overlapping objects of the
      same type

    A single camera sensor can be configured to provide any or all of these
    data at once, ensuring they all align to the same perspective.
    """

    @staticmethod
    def extract_bboxes(semantic, instance, classes):
        """
        Analyzes the given semantic annotation and instance annotation images
        for its object bounding boxes. The identified objects are returned as
        a list of dictionaries containing their bounding box corners, class
        of object according to the corresponding color in the semantic
        annotations and the provided class mapping, and the color of the
        object in the instance annotation.

        Args:
            semantic (:class:`.Image`): The image containing semantic
                                        annotation information
            instance (:class:`.Image`): The image containing instance
                                        annotation information
            classes (dict): A mapping of colors to their class names to
                            identify object types based on the semantic
                            annotation information. The keys in this dictionary
                            are the respective colors expressed as a 24bit
                            integer, i.e. `r * 256^2 + g * 256 + b`.

        Returns:
            A list of bounding boxes specified as dictionaries. One example
            bounding box dictionary contains:

            .. code-block:: python

                'bbox': [minx, miny, maxx, maxy],
                'color': [233, 11, 15],
                'class': ['CAR'],

            Where minx, miny, maxx, maxy specify the corners of the bounding
            box, color contains the RGB color of the object in the instance
            annotations, and class the object type identified through the
            given class mapping.
        """

        semantic = np.array(semantic)
        instance = np.array(instance)

        if semantic.shape != instance.shape:
            raise BNGValueError('Provided semantic and instance annotation '
                                'images have different sizes.')

        bboxes = {}

        for y in range(semantic.shape[0]):
            for x in range(semantic.shape[1]):
                color = instance[y, x]
                color_key = color[0] * 256 * 256 + color[1] * 256 + color[2]
                if color_key == 0:
                    continue

                clazz = semantic[y, x]
                clazz_key = clazz[0] * 256 * 256 + clazz[1] * 256 + clazz[2]

                if classes[clazz_key] == 'BACKGROUND':
                    continue

                if color_key in bboxes:
                    entry = bboxes[color_key]
                    bbox = entry['bbox']
                    bbox[0] = min(bbox[0], x)
                    bbox[1] = min(bbox[1], y)
                    bbox[2] = max(bbox[2], x)
                    bbox[3] = max(bbox[3], y)
                else:
                    entry = {
                        'bbox': [x, y, x, y],
                        'class': classes[clazz_key],
                        'color': [*color],
                    }
                    bboxes[color_key] = entry

        ret = []
        for _, v in bboxes.items():
            ret.append(v)
        return ret

    @staticmethod
    def export_bbox_xml(bboxes, folder=None, filename=None, path=None,
                        database=None, size=None):
        """
        Exports the given list of bounding boxes to the Pascal-VOC XML
        standard. The bounding box list is expected to be in the format that
        `extract_bboxes` returns. Additional properties to this function
        correspond to tags in the Pascal-VOC standard.

        Args:
            bboxes (list): The list of bounding boxes the export. Consult the
                           documentation of `extract_bboxes` for information
                           on its expected structure.
            folder (str): Contents of the `<folder>` tag. Optional.
            filename (str): Contents of the `<filename>` tag. Optional.
            path (str): Contents of the `<path>` tag. Optional.
            database (str): Contents of the `<database>` tag. Optional.
            size (tuple): Contents of the `<size>` tag. It's expected to be a
                          tuple of the image width, height, and depth.
                          Optional.

        Returns:
            XML string encoding the given list of bounding boxes according to
            Pascal-VOC.
        """
        root = Element('annotation')

        if folder:
            folder_elem = SubElement(root, 'folder')
            folder_elem.text = folder

        if filename:
            file_elem = SubElement(root, 'filename')
            file_elem.text = filename

        if path:
            path_elem = SubElement(root, 'path')
            path_elem.text = path

        if database:
            source = SubElement(root, 'source')
            database_elem = SubElement(source, 'database')
            database_elem.text = database

        if size:
            size_elem = SubElement(root, 'size')

            width = SubElement(size_elem, 'width')
            width.text = str(size[0])

            height = SubElement(size_elem, 'height')
            height.text = str(size[1])

            depth = SubElement(size_elem, 'depth')
            depth.text = str(size[2])

        segmented = SubElement(root, 'segmented')
        segmented.text = '0'

        for idx, bbox in enumerate(bboxes):
            object_elem = SubElement(root, 'object')
            name = SubElement(object_elem, 'name')
            name.text = '{}_{}'.format(bbox['class'], idx)

            pose = SubElement(object_elem, 'pose')
            pose.text = 'Unspecified'

            truncated = SubElement(object_elem, 'truncated')
            truncated.text = '0'

            difficult = SubElement(object_elem, 'difficult')
            difficult.text = '0'

            bndbox = SubElement(object_elem, 'bndbox')

            xmin = SubElement(bndbox, 'xmin')
            xmin.text = str(bbox['bbox'][0])

            ymin = SubElement(bndbox, 'ymin')
            ymin.text = str(bbox['bbox'][1])

            xmax = SubElement(bndbox, 'xmax')
            xmax.text = str(bbox['bbox'][2])

            ymax = SubElement(bndbox, 'ymax')
            ymax.text = str(bbox['bbox'][3])

        ret = ElementTree.tostring(root, 'utf-8')
        ret = minidom.parseString(ret)
        return ret.toprettyxml(indent='  ')

    @staticmethod
    def draw_bboxes(bboxes, color, width=3, font='arial.ttf', fontsize=14):
        """
        Draws the given list of bounding boxes onto the given image. The boxes
        are drawn with the given width of outlines in pixels and the given font
        and size configuration. The given image is not directly modified and
        the boxes are drawn onto a copy. The bboxes are expected to be in the
        format returned by `extract_bboxes`.

        Args:
            bboxes (list): List of bounding boxes to draw. Consult the
                           documentation of `extract_bboxes` for information
                           on the structure of this list.
            color (:class:`.Image`): The image to draw bounding boxes on. Will
                                     be copied and not modified in place.
            width (int): The width of bounding box outlines in pixels.
            font (str): A string specifying the font bounding box labels are
                        supposed to have.
            fontsize (int): The font size to draw labels with.

        Returns:
            A `:class:`.Image` that is a copy of the given image with bounding
            boxes drawn onto it.
        """

        color = color.copy()
        draw = ImageDraw.Draw(color)

        font = ImageFont.truetype(font=font, size=fontsize)

        for idx, bbox in enumerate(bboxes):
            bbox_color = bbox['color']
            bbox_color = (bbox_color[0], bbox_color[1], bbox_color[2])
            bbox_corners = bbox['bbox']
            draw.rectangle(bbox_corners, outline=bbox_color, width=width)

            text = '{}_{}'.format(bbox['class'], idx)
            text_pos = (bbox_corners[0], bbox_corners[3])
            text_anchor = 'lt'

            if text_pos[1] > color.size[1] * 0.9:
                text_pos = (bbox_corners[0], bbox_corners[1])
                text_anchor = 'lb'

            draw.text(text_pos, text, fill='#FFFFFF', stroke_width=2,
                      stroke_fill='#000000', font=font, anchor=text_anchor)

        return color

    def __init__(self, pos=(0, 0, 3), direction=(0, -1, 0),
                 fov=(10, 10), resolution=(200, 200), near_far=(NEAR, FAR),
                 colour=False, depth=False,
                 depth_inverse=False, annotation=False, instance=False, shmem=True,
                 up=(0, 0, 1), requested_update_time=0.1, update_priority=0.0,
                 visualised=True, static=False, snapping_desired=False, force_inside_triangle=False):
        """
        Creates a camera sensor.

        Args:
            pos (tuple): (X, Y, Z) Coordinate triplet specifying the position of the sensor, in world space.
            direction (tuple): (X, Y, Z) Coordinate triplet specifying the forward direction of the sensor.
            fov (tuple): (X, Y) The sensor field of view parameters.
            resolution (tuple): (X, Y) The resolution of the sensor images.
            near_far (tuple): (X, Y) The sensor near and far plane distances.
            colour (bool): A flag which indicates if this sensor should render colour data.
            depth (bool): A flag which indicates if this sensor should render depth data.
            depth_inverse (bool): A flag which indicates if the depth values should be shown white->black or black->white, as distance increases.
            annotation (bool): A flag which indicates if this sensor should render annotation data.
            instance (bool): A flag which indicates if this sensor should render instance annotations, rather than semantic annotations.
            shmem (bool): A flag which indicates if we should use shared memory to send/recieve the sensor readings data.
            up (tuple): (X, Y, Z) Coordinate triplet specifying the up direction of the sensor.
            requested_update_time (float): The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
            update_priority (float): The priority which the sensor should ask for new readings. lowest -> 0, highest -> 1.
            visualised (bool): A flag which indicates if this LiDAR sensor should appear visualised or not.
            static (bool): A flag which indicates whether this sensor should be static (fixed position), or attached to a vehicle.
            snapping_desired (bool): A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle (not used for static sensors).
            force_inside_triangle (bool): A flag which indicates if the sensor should be forced inside the nearest vehicle triangle (not used for static sensors).
        """
        if isinstance(fov, numbers.Number):
            fov = (fov, fov)
        camera_gen = lambda name, bng, vehicle: AutoCamera(
            name, bng, vehicle, pos, direction, fov, resolution, near_far, colour, depth, depth_inverse, annotation,
            instance, shmem, up, requested_update_time, update_priority, visualised, static, snapping_desired,
            force_inside_triangle)
        super().__init__(camera_gen)


class AutoCamera:
    def __init__(
            self, name, bng, vehicle=None, pos=(0, 0, 3), direction=(0, -1, 0),
            fov=(10, 10), resolution=(200, 200), near_far=(NEAR, FAR),
            colour=False, depth=False,
            depth_inverse=False, annotation=False, instance=False, shmem=True,
            up=(0, 0, 1), requested_update_time=0.1, update_priority=0.0,
            visualised=True, static=False, snapping_desired=False, force_inside_triangle=False):
        """
        Creates a camera sensor.

        Args:
            name (str): A unique name for this camera sensor.
            bng (BeamNGpy): The BeamNGpy instance, with which to communicate to the simulation.
            vehicle (Vehicle class): The vehicle to which this sensor should be attached, if any.
            pos (tuple): (X, Y, Z) Coordinate triplet specifying the position of the sensor, in world space.
            direction (tuple): (X, Y, Z) Coordinate triplet specifying the forward direction of the sensor.
            fov (tuple): (X, Y) The sensor field of view parameters.
            resolution (tuple): (X, Y) The resolution of the sensor images.
            near_far (tuple): (X, Y) The sensor near and far plane distances.
            colour (bool): A flag which indicates if this sensor should render colour data.
            depth (bool): A flag which indicates if this sensor should render depth data.
            depth_inverse (bool): A flag which indicates if the depth values should be shown white->black or black->white, as distance increases.
            annotation (bool): A flag which indicates if this sensor should render annotation data.
            instance (bool): A flag which indicates if this sensor should render instance annotations, rather than semantic annotations.
            shmem (bool): A flag which indicates if we should use shared memory to send/recieve the sensor readings data.
            up (tuple): (X, Y, Z) Coordinate triplet specifying the up direction of the sensor.
            requested_update_time (float): The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
            update_priority (float): The priority which the sensor should ask for new readings. lowest -> 0, highest -> 1.
            visualised (bool): A flag which indicates if this LiDAR sensor should appear visualised or not.
            static (bool): A flag which indicates whether this sensor should be static (fixed position), or attached to a vehicle.
            snapping_desired (bool): A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle (not used for static sensors).
            force_inside_triangle (bool): A flag which indicates if the sensor should be forced inside the nearest vehicle triangle (not used for static sensors).
        """

        self.logger = getLogger(f'{LOGGER_ID}.AutoCamera')
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name
        self.resolution = resolution
        self.near_far = near_far
        self.depth_inverse = depth_inverse
        self.colour = colour
        self.annotation = annotation
        self.depth = depth
        self.instance = instance

        # Set up the shared memory for this sensor, if requested.
        self.shmem = shmem
        self.colour_handle = None
        self.colour_shmem = None
        self.annotation_handle = None
        self.annotation_shmem = None
        self.depth_handle = None
        self.depth_shmem = None
        buffer_size = -1
        if shmem:
            self.logger.debug('Camera - Initializing shared memory.')
            pid = os.getpid()
            buffer_size = resolution[0] * resolution[1] * 4
            if colour:
                self.colour_handle = '{}.{}.{}.colour'.format(pid, '', name)
                self.colour_shmem = mmap.mmap(0, buffer_size, self.colour_handle)
                self.logger.debug('Camera - Bound shared memory for colour: 'f'{self.colour_handle}')

            if annotation:
                self.annotation_handle = '{}.{}.{}.annotations'.format(pid, '', name)
                self.annotation_shmem = mmap.mmap(0, buffer_size, self.annotation_handle)
                self.logger.debug('Camera - Bound shared memory for annotation: 'f'{self.annotation_handle}')

            if depth:
                self.depth_handle = '{}.{}.{}.depth'.format(pid, '', name)
                self.depth_shmem = mmap.mmap(0, buffer_size, self.depth_handle)
                self.logger.debug('Camera - Bound shared memory for depth: 'f'{self.depth_handle}')

        # Create and initialise the camera in the simulation.
        bng.open_auto_camera(
            name, vehicle, requested_update_time, update_priority, self.resolution, fov, near_far,
            pos, direction, up, shmem, self.colour_handle, buffer_size, self.annotation_handle, buffer_size,
            self.depth_handle, buffer_size, colour, annotation, depth,
            instance, visualised, static, snapping_desired, force_inside_triangle)
        self.logger.debug('Camera - sensor created: 'f'{self.name}')

    def _convert_to_image(self, raw_data, width, height, channels, data_type):
        """
        Converts raw image data from the simulator into image format.

        Args:
            raw_data (array): The 1D buffer to be processed.
            width (int): The width of the image to be rendered.
            height (int): The height of the image to be rendered.
            channels(int): The number of channels in the data, here either 4 for RGBA or 1 for depth data. 
            data_type: The type of data, eg np.uint8, np.float32.

        Returns:
            (PIL image object): The processed image. 
        """

        # Convert to a numpy array.
        decoded = np.frombuffer(bytes(raw_data), dtype=data_type)

        # Re-shape the array, based on the number of channels present in the data.
        if channels > 1:
            decoded = decoded.reshape(height, width, channels)
        else:
            decoded = decoded.reshape(height, width)

        # Convert to image format.
        return ImageOps.flip(Image.fromarray(decoded))

    def _depth_buffer_processing(self, raw_depth_values):
        """
        Converts raw depth buffer data to visually-clear intensity values in the range [0, 255].
        We process the data so that small changes in distance are better shown, rather than just using linear interpolation.

        Args:
            raw_depth_values (array): The raw 1D buffer of depth values.

        Returns:
            (array): The processed intensity values in the range [0, 255].
        """

        # Sort the depth values, and cache the sorting map.
        sort_index = np.argsort(raw_depth_values)
        s_data = []
        for i in range(len(raw_depth_values)):
            s_data.append(raw_depth_values[sort_index[i]])

        # Compute an array of unique depth values, sensitive to some epsilon.
        size = len(s_data)
        unique = []
        unique.append(s_data[0])
        current = s_data[0]
        for i in range(1, size):
            if abs(s_data[i] - current) > 0.01:
                unique.append(s_data[i])
                current = s_data[i]

        # Distribute (mark) the individual intensity values throughout the sorted unique distance array.
        intensity_marks = []
        intensity_marks.append(0)
        i_reciprocal = 1.0 / 255.0
        for i in range(254):
            intensity_marks.append(unique[math.floor(len(unique) * i * i_reciprocal)])
        intensity_marks.append(1e12)

        # In the sorted depth values array, convert the depth value array into intensity values.
        depth_intensity_sorted = np.zeros((size))
        im_index = 0
        for i in range(size):
            depth_intensity_sorted[i] = im_index
            if s_data[i] >= intensity_marks[im_index + 1]:
                im_index = im_index + 1

        # Re-map the depth values back to their original order.
        depth_intensity = np.zeros((size))
        for i in range(len(raw_depth_values)):
            if self.depth_inverse:
                depth_intensity[sort_index[i]] = 255 - depth_intensity_sorted[i]
            else:
                depth_intensity[sort_index[i]] = depth_intensity_sorted[i]

        return depth_intensity

    def _binary_to_image(self, binary):
        """
        Converts the binary string data from the simulator, which contains the data buffers for colour, annotations, and depth, into images.

        Args:
            binary (binary string): The raw readings data, as a binary string.

        Returns:
            (dict): A dictionary containing the processed images.
        """

        width = self.resolution[0]
        height = self.resolution[1]

        processed_readings = dict(type='Camera')

        if self.colour:
            colour = []
            for i in range(len(binary['colour'])):
                colour.append(np.uint8(binary['colour'][i]))
            processed_readings['colour'] = self._convert_to_image(colour, width, height, 4, np.uint8)

        if self.annotation:
            annotation = []
            for i in range(len(binary['annotation'])):
                annotation.append(np.uint8(binary['annotation'][i]))
            processed_readings['annotation'] = self._convert_to_image(annotation, width, height, 4, np.uint8)

        if self.depth:
            depth = np.zeros(int(len(binary['depth']) / 4))
            ctr = 0
            for i in range(0, int(len(binary['depth'])), 4):
                depth[ctr] = struct.unpack('f', binary['depth'][i:i + 4])[0]
                ctr = ctr + 1
            processed_values = self._depth_buffer_processing(depth)
            reshaped_data = processed_values.reshape(height, width)
            processed_readings['depth'] = ImageOps.flip(Image.fromarray(reshaped_data))

        return processed_readings

    def remove(self):
        """
        Removes this sensor from the simulation.
        """

        # Remove any shared memory binding which this sensor is using.
        if self.shmem:
            if self.colour:
                self.logger.debug('Camera - Unbinding shared memory for colour: 'f'{self.colour_handle}')
                self.colour_shmem.close()

            if self.annotation:
                self.logger.debug('Camera - Unbinding shared memory for annotation: 'f'{self.annotation_handle}')
                self.annotation_shmem.close()

            if self.depth:
                self.logger.debug('Camera - Unbinding shared memory for depth: 'f'{self.depth_handle}')
                self.depth_shmem.close()

        # Remove this sensor from the simulation.
        self.bng.close_auto_camera(self.name, self.instance)
        self.logger.debug('Camera - sensor removed: 'f'{self.name}')

    def poll(self):
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            (dict): The processed images. 
        """

        # Send and receive a request for readings data from this sensor.
        raw_readings = self.bng.poll_auto_camera(
            self.name, self.shmem, self.instance)['data']

        self.logger.debug('Camera - raw sensor readings received from simulation: 'f'{self.name}')

        # Decode the raw sensor readings into image data. This is handled differently, depending on whether shared memory is used or not.
        width = self.resolution[0]
        height = self.resolution[1]
        buffer_size = width * height * 4
        images = dict(type='Camera')
        if self.shmem:
            # CASE 1: We are using shared memory.
            if self.colour:
                if 'colour' in raw_readings.keys():
                    self.colour_shmem.seek(0)
                    colour_d = self.colour_shmem.read(buffer_size)
                    colour_d = np.frombuffer(colour_d, dtype=np.uint8)
                    colour_d = colour_d.reshape(height, width, 4)
                    images['colour'] = ImageOps.flip(Image.fromarray(colour_d))
                else:
                    self.logger.error(
                        'Camera - Colour buffer failed to render. Check that you are not running on low settings.')

            if self.annotation:
                if 'annotation' in raw_readings.keys():
                    self.annotation_shmem.seek(0)
                    annotate_d = self.annotation_shmem.read(buffer_size)
                    annotate_d = np.frombuffer(annotate_d, dtype=np.uint8)
                    annotate_d = annotate_d.reshape(height, width, 4)
                    images['annotation'] = ImageOps.flip(Image.fromarray(annotate_d))
                else:
                    self.logger.error(
                        'Camera - Annotation buffer failed to render. Check that you are not running on low settings.')

            if self.depth:
                if 'depth' in raw_readings.keys():
                    self.depth_shmem.seek(0)
                    depth_values = self.depth_shmem.read(buffer_size)
                    depth_values = np.frombuffer(bytes(depth_values), dtype=np.float32)
                    depth_values = self._depth_buffer_processing(depth_values)
                    depth_values = depth_values.reshape(height, width)
                    depth_values = np.uint8(depth_values)
                    images['depth'] = ImageOps.flip(Image.fromarray(depth_values))
                else:
                    self.logger.error(
                        'Camera - Depth buffer failed to render. Check that you are not running on low settings.')

            self.logger.debug('Camera - sensor readings read from shared memory and processed: 'f'{self.name}')
        else:

            # CASE 2: We are not using shared memory. The data is coming back across the socket.
            images = self._binary_to_image(raw_readings)

            self.logger.debug('Camera - raw sensor readings converted from base64 to image format: 'f'{self.name}')
        return images

    def send_ad_hoc_poll_request(self):
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            (int): A unique Id number for the ad-hoc request.
        """

        self.logger.debug('Camera - ad-hoc polling request sent: 'f'{self.name}')
        return self.bng.send_ad_hoc_request_auto_camera(self.name)['data']

    def is_ad_hoc_poll_request_ready(self, request_id):
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (bool): A flag which indicates if the ad-hoc polling request is complete.
        """

        self.logger.debug('Camera - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self.bng.is_ad_hoc_poll_request_ready_auto_camera(request_id)

    def collect_ad_hoc_poll_request(self, request_id):
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (dict): The readings data.
        """

        # Obtain the raw readings (as a binary string) from the simulator, for this ad-hoc polling request.
        raw_readings = self.bng.collect_ad_hoc_poll_request_auto_camera(request_id)['data']

        # Format the binary string data from the simulator.
        return self._binary_to_image(raw_readings)

    def get_requested_update_time(self):
        """
        Gets the current 'requested update time' value for this sensor.

        Returns:
            (float): The requested update time.
        """

        return self.bng.get_camera_requested_update_time(self.name)['data']

    def get_update_priority(self):
        """
        Gets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, highest to lowest.

        Returns:
            (float): The update priority value.
        """

        return self.bng.get_camera_update_priority(self.name)['data']

    def get_position(self):
        """
        Gets the current world-space position of this sensor.

        Returns:
            (list): The sensor position.
        """
        table = self.bng.get_camera_sensor_position(self.name)['data']
        return [table['x'], table['y'], table['z']]

    def get_direction(self):
        """
        Gets the current direction vector of this sensor.

        Returns:
            (list): The sensor direction.
        """
        table = self.bng.get_camera_sensor_direction(self.name)['data']
        return [table['x'], table['y'], table['z']]

    def get_max_pending_requests(self):
        """
        Gets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Returns:
            (int): The max pending requests value.
        """

        return self.bng.get_camera_max_pending_gpu_requests(self.name)['data']

    def set_requested_update_time(self, requested_update_time):
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time (float): The new requested update time.
        """

        self.bng.set_camera_requested_update_time(self.name, requested_update_time)

    def set_update_priority(self, update_priority):
        """
        Sets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, , highest to lowest.

        Args:
            update_priority (float): The new update priority value.
        """

        self.bng.set_camera_update_priority(self.name, update_priority)

    def set_max_pending_requests(self, max_pending_requests):
        """
        Sets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Args:
            max_pending_requests (int): The new max pending requests value.
        """

        self.bng.set_camera_max_pending_gpu_requests(self.name, max_pending_requests)
