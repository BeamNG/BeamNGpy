"""
An interactive, automated camera sensor, which can produce regular colour images, depth images, or annotation images.
This sensor can be attached to a vehicle, or can be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.
"""

import math
import mmap
import os
import struct
import numpy as np
from logging import DEBUG, getLogger
from beamngpy.beamngcommon import LOGGER_ID, BNGValueError
from xml.dom import minidom
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement
from PIL import Image, ImageDraw, ImageFont, ImageOps

class Camera:

    @staticmethod
    def extract_bounding_boxes(semantic_data, instance_data, classes):
        """
        Analyzes the given semantic annotation and instance annotation images for its object bounding boxes. The identified objects are returned as
        a list of dictionaries containing their bounding box corners, class of object according to the corresponding colour in the semantic
        annotations and the given class mapping, and the colour of the object in the instance annotation.

        Args:
            semantic_data (:class:`.Image`): The image containing semantic annotation information.
            instance_data (:class:`.Image`): The image containing instance annotation information.
            classes (dict): A mapping of colours to their class names to identify object types based on the semantic annotation information. The keys in
                this dictionary are the respective colours expressed as a 24-bit integer, i.e. [r * 256^2 + g * 256 + b].

        Returns:
            A list of bounding boxes specified as dictionaries. Example: 'bbox': [min_x, min_y, max_x, max_y], 'color': [233, 11, 15], 'class': ['CAR'],
            where min_x, min_y, max_x, max_y mark the corners of the bounding box, colour contains the RGB colour of the object in the instance
            annotations, and class the object type identified through the given class mapping.
        """

        semantic_data = np.array(semantic_data)
        instance_data = np.array(instance_data)

        if semantic_data.shape != instance_data.shape:
            raise BNGValueError('Error - The given semantic and instance annotation images have different resolutions.')

        bounding_boxes = {}
        for y in range(semantic_data.shape[0]):
            for x in range(semantic_data.shape[1]):
                colour = instance_data[y, x]
                colour_key = colour[0] * 65536 + colour[1] * 256 + colour[2]
                if colour_key == 0:
                    continue

                clazz = semantic_data[y, x]
                clazz_key = clazz[0] * 65536 + clazz[1] * 256 + clazz[2]
                if classes[clazz_key] == 'BACKGROUND':
                    continue

                if colour_key in bounding_boxes:
                    entry = bounding_boxes[colour_key]
                    box = entry['bbox']
                    box[0] = min(box[0], x)
                    box[1] = min(box[1], y)
                    box[2] = max(box[2], x)
                    box[3] = max(box[3], y)
                else:
                    entry = {
                        'bbox': [x, y, x, y],
                        'class': classes[clazz_key],
                        'color': [*colour],
                    }
                    bounding_boxes[colour_key] = entry

        box_list = []
        for _, v in bounding_boxes.items():
            box_list.append(v)
        return box_list

    @staticmethod
    def draw_bounding_boxes(bounding_boxes, colour, width=3, font='arial.ttf', font_size=14):
        """
        Draws the given list of bounding boxes onto the given image. The boxes are drawn with the given width of outlines in pixels and the given font
        and size configuration. NOTE: The given image is not directly modified and the boxes are drawn onto a copy.

        Args:
            bounding_boxes (list): List of bounding boxes to draw.
            colour (:class:`.Image`): The image to draw the bounding boxes on.
            width (int): The width of bounding box outlines in pixels.
            font (str): A string specifying the font which bounding box labels will have.
            fontsize (int): The font size used when drawing labels.

        Returns:
            A `:class:`.Image` that is a copy of the given image with bounding boxes drawn onto it.
        """

        colour = colour.copy()
        draw = ImageDraw.Draw(colour)

        font = ImageFont.truetype(font=font, size=font_size)

        for i, box in enumerate(bounding_boxes):
            box_colour = box['color']
            box_colour = (box_colour[0], box_colour[1], box_colour[2])
            box_corners = box['bbox']
            draw.rectangle(box_corners, outline=box_colour, width=width)

            text = '{}_{}'.format(box['class'], i)
            text_pos = (box_corners[0], box_corners[3])
            text_anchor = 'lt'

            if text_pos[1] > colour.size[1] * 0.9:
                text_pos = (box_corners[0], box_corners[1])
                text_anchor = 'lb'

            draw.text(text_pos, text, fill='#FFFFFF', stroke_width=2, stroke_fill='#000000', font=font, anchor=text_anchor)

        return colour

    @staticmethod
    def export_bounding_boxes_xml(bounding_boxes, folder=None, filename=None, path=None, database=None, size=None):
        """
        Exports the given list of bounding boxes to the Pascal-VOC XML standard. Additional properties to this function correspond to tags in the
        Pascal-VOC standard.

        Args:
            bounding_boxes (list): The list of bounding boxes to export.
            folder (str): Contents of the 'folder' tag.
            filename (str): Contents of the 'filename' tag.
            path (str): Contents of the 'path' tag.
            database (str): Contents of the 'database' tag.
            size (tuple): Contents of the 'size tag. It's expected to be a tuple of the image width, height, and depth.

        Returns:
            XML string encoding of the given list of bounding boxes according to Pascal-VOC.
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

        for i, bbox in enumerate(bounding_boxes):
            object_elem = SubElement(root, 'object')
            name = SubElement(object_elem, 'name')
            name.text = '{}_{}'.format(bbox['class'], i)
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

    def __init__(self, name, bng, vehicle=None, requested_update_time=0.1, update_priority=0.0, pos=(0, 0, 3), dir=(0, -1, 0), up=(0, 0, 1),
        resolution=(512, 512), field_of_view_y=70, near_far_planes=(0.05, 100.0), is_using_shared_memory=True, is_render_colours=True,
        is_render_annotations=True, is_render_instance=False, is_render_depth=True, is_depth_inverted=False, is_visualised=True, is_static=False,
        is_snapping_desired=False, is_force_inside_triangle=False):
        """
        Creates a camera sensor.

        Args:
            name (str): A unique name for this camera sensor.
            bng (BeamNGpy): The BeamNGpy instance, with which to communicate to the simulation.
            vehicle (Vehicle class): The vehicle to which this sensor should be attached, if any.
            requested_update_time (float): The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
            update_priority (float): The priority which the sensor should ask for new readings. lowest -> 0, highest -> 1.
            pos (tuple): (X, Y, Z) Coordinate triplet specifying the position of the sensor, in world space.
            dir (tuple): (X, Y, Z) Coordinate triplet specifying the forward direction of the sensor.
            up (tuple): (X, Y, Z) Coordinate triplet specifying the up direction of the sensor.
            resolution (tuple): (X, Y) The resolution of the sensor images.
            field_of_view_y (float): The sensor vertical field of view parameters.
            near_far_planes (tuple): (X, Y) The sensor near and far plane distances.
            is_using_shared_memory (bool): A flag which indicates if we should use shared memory to send/recieve the sensor readings data.
            is_render_colours (bool): A flag which indicates if this sensor should render colour data.
            is_render_annotations (bool): A flag which indicates if this sensor should render semantic annotation data.
            is_render_instance (bool): A flag which indicates if this sensor should render instance annotation data.
            is_render_depth (bool): A flag which indicates if this sensor should render depth data.
            is_depth_inverted (bool): A flag which indicates if the depth values should be shown white->black or black->white, as distance increases.
            is_visualised (bool): A flag which indicates if this LiDAR sensor should appear visualised or not.
            is_static (bool): A flag which indicates whether this sensor should be static (fixed position), or attached to a vehicle.
            is_snapping_desired (bool): A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle (not used for static sensors).
            is_force_inside_triangle (bool): A flag which indicates if the sensor should be forced inside the nearest vehicle triangle (not used for static sensors).
        """
        self.logger = getLogger(f'{LOGGER_ID}.Camera')
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name
        self.resolution = resolution
        self.near_far_planes = near_far_planes
        self.is_depth_inverted = is_depth_inverted
        self.is_render_colours = is_render_colours
        self.is_render_annotations = is_render_annotations
        self.is_render_instance = is_render_instance
        self.is_render_depth = is_render_depth

        # Set up the shared memory for this sensor, if requested.
        self.is_using_shared_memory = is_using_shared_memory
        self.colour_handle = None
        self.colour_shmem = None
        self.annotation_handle = None
        self.annotation_shmem = None
        self.instance_handle = None
        self.instance_shmem = None
        self.depth_handle = None
        self.depth_shmem = None
        buffer_size = -1
        if is_using_shared_memory:
            self.logger.debug('Camera - Initializing shared memory.')
            pid = os.getpid()
            buffer_size = resolution[0] * resolution[1] * 4
            if is_render_colours:
                self.colour_handle = '{}.{}.{}.colour'.format(pid, '', name)
                self.colour_shmem = mmap.mmap(0, buffer_size, self.colour_handle)
                self.logger.debug('Camera - Bound shared memory for colour: 'f'{self.colour_handle}')

            if is_render_annotations:
                self.annotation_handle = '{}.{}.{}.annotations'.format(pid, '', name)
                self.annotation_shmem = mmap.mmap(0, buffer_size, self.annotation_handle)
                self.logger.debug('Camera - Bound shared memory for semantic annotations: 'f'{self.annotation_handle}')

            if is_render_instance:
                self.instance_handle = '{}.{}.{}.instance'.format(pid, '', name)
                self.instance_shmem = mmap.mmap(0, buffer_size, self.instance_handle)
                self.logger.debug('Camera - Bound shared memory for instance annotations: 'f'{self.instance_handle}')

            if is_render_depth:
                self.depth_handle = '{}.{}.{}.depth'.format(pid, '', name)
                self.depth_shmem = mmap.mmap(0, buffer_size, self.depth_handle)
                self.logger.debug('Camera - Bound shared memory for depth: 'f'{self.depth_handle}')

        # Create and initialise the camera in the simulation.
        bng.open_camera(name, vehicle, requested_update_time, update_priority, self.resolution, field_of_view_y, near_far_planes, pos, dir, up,
            is_using_shared_memory, self.colour_handle, buffer_size, self.annotation_handle, buffer_size, self.depth_handle, buffer_size, is_render_colours,
            is_render_annotations, is_render_instance, is_render_depth, is_visualised, is_static, is_snapping_desired, is_force_inside_triangle)
        self.logger.debug('Camera - sensor created: 'f'{self.name}')

    def _convert_to_image(self, raw_data, width, height, channels, data_type):
        """
        Converts raw image data from the simulator into image format.

        Args:
            raw_data (array): The 1D buffer to be processed.
            width (int): The width of the image to be rendered.
            height (int): The height of the image to be rendered.
            channels (int): The number of channels in the data, here either 4 for RGBA or 1 for depth data.
            data_type: The type of data, eg 'np.uint8', 'np.float32'.

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
        return ImageOps.mirror(ImageOps.flip(Image.fromarray(decoded)))

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
            if self.is_depth_inverted:
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

        if self.is_render_colours:
            colour = []
            for i in range(len(binary['colour'])):
                colour.append(np.uint8(binary['colour'][i]))
            processed_readings['colour'] = self._convert_to_image(colour, width, height, 4, np.uint8)

        if self.is_render_annotations:
            annotation = []
            for i in range(len(binary['annotation'])):
                annotation.append(np.uint8(binary['annotation'][i]))
            processed_readings['annotation'] = self._convert_to_image(annotation, width, height, 4, np.uint8)

        if self.is_render_instance:
            instance = []
            for i in range(len(binary['instance'])):
                instance.append(np.uint8(binary['instance'][i]))
            processed_readings['instance'] = self._convert_to_image(annotation, width, height, 4, np.uint8)

        if self.is_render_depth:
            depth = np.zeros(int(len(binary['depth']) / 4))
            ctr = 0
            for i in range(0, int(len(binary['depth'])), 4):
                depth[ctr] = struct.unpack('f', binary['depth'][i:i + 4])[0]
                ctr = ctr + 1
            processed_values = self._depth_buffer_processing(depth)
            reshaped_data = processed_values.reshape(height, width)
            processed_readings['depth'] = ImageOps.mirror(ImageOps.flip(Image.fromarray(reshaped_data)))

        return processed_readings

    def remove(self):
        """
        Removes this sensor from the simulation.
        """
        # Remove any shared memory binding which this sensor is using.
        if self.is_using_shared_memory:
            if self.is_render_colours:
                self.logger.debug('Camera - Unbinding shared memory for colour: 'f'{self.colour_handle}')
                self.colour_shmem.close()

            if self.is_render_annotations:
                self.logger.debug('Camera - Unbinding shared memory for semantic annotations: 'f'{self.annotation_handle}')
                self.annotation_shmem.close()

            if self.is_render_instance:
                self.logger.debug('Camera - Unbinding shared memory for instance annotations: 'f'{self.instance_handle}')
                self.instance_shmem.close()

            if self.is_render_depth:
                self.logger.debug('Camera - Unbinding shared memory for depth: 'f'{self.depth_handle}')
                self.depth_shmem.close()

        # Remove this sensor from the simulation.
        self.bng.close_camera(self.name)
        self.logger.debug('Camera - sensor removed: 'f'{self.name}')

    def poll(self):
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            (dict): The processed images.
        """
        # Send and receive a request for readings data from this sensor.
        raw_readings = self.bng.poll_camera(self.name, self.is_using_shared_memory)['data']

        self.logger.debug('Camera - raw sensor readings received from simulation: 'f'{self.name}')

        # Decode the raw sensor readings into image data. This is handled differently, depending on whether shared memory is used or not.
        width = self.resolution[0]
        height = self.resolution[1]
        buffer_size = width * height * 4
        images = dict(type='Camera')
        if self.is_using_shared_memory:

            # CASE 1: We are using shared memory.
            if self.is_render_colours:
                if 'colour' in raw_readings.keys():
                    self.colour_shmem.seek(0)
                    colour_d = self.colour_shmem.read(buffer_size)
                    colour_d = np.frombuffer(colour_d, dtype=np.uint8)
                    colour_d = colour_d.reshape(height, width, 4)
                    images['colour'] = ImageOps.mirror(ImageOps.flip(Image.fromarray(colour_d)))
                else:
                    self.logger.error('Camera - Colour buffer failed to render. Check that you are not running on low settings.')

            if self.is_render_annotations:
                if 'annotation' in raw_readings.keys():
                    self.annotation_shmem.seek(0)
                    annotate_d = self.annotation_shmem.read(buffer_size)
                    annotate_d = np.frombuffer(annotate_d, dtype=np.uint8)
                    annotate_d = annotate_d.reshape(height, width, 4)
                    images['annotation'] = ImageOps.mirror(ImageOps.flip(Image.fromarray(annotate_d)))
                else:
                    self.logger.error('Camera - Annotation buffer failed to render. Check that you are not running on low settings.')

            if self.is_render_depth:
                if 'depth' in raw_readings.keys():
                    self.depth_shmem.seek(0)
                    depth_values = self.depth_shmem.read(buffer_size)
                    depth_values = np.frombuffer(bytes(depth_values), dtype=np.float32)
                    depth_values = self._depth_buffer_processing(depth_values)
                    depth_values = depth_values.reshape(height, width)
                    depth_values = np.uint8(depth_values)
                    images['depth'] = ImageOps.mirror(ImageOps.flip(Image.fromarray(depth_values)))
                else:
                    self.logger.error('Camera - Depth buffer failed to render. Check that you are not running on low settings.')

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
        return self.bng.send_ad_hoc_request_camera(self.name)['data']

    def is_ad_hoc_poll_request_ready(self, request_id):
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (bool): A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug('Camera - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self.bng.is_ad_hoc_poll_request_ready_camera(request_id)

    def collect_ad_hoc_poll_request(self, request_id):
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (dict): The readings data.
        """
        # Obtain the raw readings (as binary strings) from the simulator, for this ad-hoc polling request.
        raw_readings = self.bng.collect_ad_hoc_poll_request_camera(request_id)['data']

        # Format the binary string data from the simulator.
        return self._binary_to_image(raw_readings)

    # TODO: Should be removed when GE-2170 is complete.
    def get_full_poll_request(self):
        """
        Gets a full camera request (semantic annotation and instance annotation data included).
        NOTE: this function blocks the simulation until the data request is completed.

        Returns:
            (dict): The camera data, as images
        """
        # Obtain the raw readings (as binary strings) from the simulator, for this ad-hoc polling request.
        raw_readings1 = self.bng.get_full_camera_request_semantic(self.name)['data']
        raw_readings1 = self._binary_to_image(raw_readings1)

        raw_readings2 = self.bng.get_full_camera_request_instance(self.name)['data']
        raw_readings2 = self._binary_to_image(raw_readings2)

        data = dict(type='data')
        data['colour'] = raw_readings1['colour']
        data['annotation'] = raw_readings1['annotation']
        data['instance'] = raw_readings2['instance']
        data['depth'] = raw_readings1['depth']

        # Format the binary string data from the simulator.
        return data

    def world_point_to_pixel(self, point):
        """
        Converts a 3D point in world space to the 2D pixel coordinate at which it is represented on this camera.
        NOTE: The pixel does not have to actually be visible on the camera image itself in order to retrieve a value; it can be obscured by geometry
        which is closer, or it can be run without respect to the near and far plane values of the camera.

        Args:
            point (tuple): The given 3D point, in world space coordinates.

        Returns:
            (list): The 2D pixel value which represents the given 3D point, on this camera.
        """
        pixel_data = self.bng.camera_world_point_to_pixel(self.name, point)['data']
        return [int(pixel_data['x']), int(pixel_data['y'])]

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
        Gets the current forward direction vector of this sensor.

        Returns:
            (list): The sensor direction.
        """
        table = self.bng.get_camera_sensor_direction(self.name)['data']
        return [table['x'], table['y'], table['z']]

    def get_up(self):
        """
        Gets the current up direction vector of this sensor.

        Returns:
            (list): The sensor direction.
        """
        table = self.bng.get_camera_sensor_up(self.name)['data']
        return [table['x'], table['y'], table['z']]

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

    def get_max_pending_requests(self):
        """
        Gets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Returns:
            (int): The max pending requests value.
        """
        return self.bng.get_camera_max_pending_gpu_requests(self.name)['data']

    def set_position(self, pos):
        """
        Sets the current world-space position for this sensor.

        Args:
            pos (tuple): The new position.
        """
        self.bng.set_camera_sensor_position(self.name, pos)

    def set_direction(self, dir):
        """
        Sets the current forward direction vector of this sensor.

        Args:
            pos (tuple): The new forward direction vector.
        """
        self.bng.set_camera_sensor_direction(self.name, dir)

    def set_up(self, up):
        """
        Sets the current up vector of this sensor.

        Args:
            pos (tuple): The new up vector.
        """
        self.bng.set_camera_sensor_up(self.name, up)

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
