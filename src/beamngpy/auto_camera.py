"""
An interactive, automated camera sensor, which can produce regular colour images, depth images, or annotation images.
This sensor can be attached to a vehicle, or can be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.
"""

import os
import mmap
import math
import numpy as np
import struct
from PIL import Image, ImageOps
from logging import DEBUG as DBG_LOG_LEVEL, getLogger
from .beamngcommon import LOGGER_ID

class Auto_Camera:

    def __init__(self, name, bng, vehicle=None, requested_update_time=0.1, update_priority=0.0, pos=(0, 0, 3), dir=(0, -1, 0), up=(0, 0, 1), 
        render_size=(200, 200), field_of_view=(10, 10), near_far_planes=(0.05, 100.0), is_using_shared_memory=True, is_render_colours=True, 
        is_render_annotations=True, is_render_depth=True, use_instance_annotations=True, is_depth_inverted=False, is_visualised=True, is_static=False, 
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
            render_size (tuple): (X, Y) The resolution of the sensor images.
            field_of_view (tuple): (X, Y) The sensor field of view parameters.
            near_far_planes (tuple): (X, Y) The sensor near and far plane distances.
            is_using_shared_memory (bool): A flag which indicates if we should use shared memory to send/recieve the sensor readings data.
            is_render_colours (bool): A flag which indicates if this sensor should render colour data.
            is_render_annotations (bool): A flag which indicates if this sensor should render annotation data.
            is_render_depth (bool): A flag which indicates if this sensor should render depth data.
            use_instance_annotations (bool): A flag which indicates if this sensor should render instance annotations, rather than semantic annotations.
            is_depth_inverted (bool): A flag which indicates if the depth values should be shown white->black or black->white, as distance increases.
            is_visualised (bool): A flag which indicates if this LiDAR sensor should appear visualised or not.
            is_static (bool): A flag which indicates whether this sensor should be static (fixed position), or attached to a vehicle.
            is_snapping_desired (bool): A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle (not used for static sensors).
            is_force_inside_triangle (bool): A flag which indicates if the sensor should be forced inside the nearest vehicle triangle (not used for static sensors).
        """

        self.logger = getLogger(f'{LOGGER_ID}.AutoCamera')
        self.logger.setLevel(DBG_LOG_LEVEL)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name
        self.render_size = render_size
        self.near_far_planes = near_far_planes
        self.is_depth_inverted = is_depth_inverted
        self.is_render_colours = is_render_colours
        self.is_render_annotations = is_render_annotations
        self.is_render_depth = is_render_depth
        self.use_instance_annotations = use_instance_annotations

        # Set up the shared memory for this sensor, if requested.
        self.is_using_shared_memory = is_using_shared_memory
        self.colour_handle = None
        self.colour_shmem = None
        self.annotation_handle = None
        self.annotation_shmem = None
        self.depth_handle = None
        self.depth_shmem = None
        buffer_size = -1
        if is_using_shared_memory:
            self.logger.debug('Camera - Initializing shared memory.')
            pid = os.getpid()
            buffer_size = render_size[0] * render_size[1] * 4
            if is_render_colours:
                self.colour_handle = '{}.{}.{}.colour'.format(pid, '', name)
                self.colour_shmem = mmap.mmap(0, buffer_size, self.colour_handle)
                self.logger.debug('Camera - Bound shared memory for colour: 'f'{self.colour_handle}')

            if is_render_annotations:
                self.annotation_handle = '{}.{}.{}.annotations'.format(pid, '', name)
                self.annotation_shmem = mmap.mmap(0, buffer_size, self.annotation_handle)
                self.logger.debug('Camera - Bound shared memory for annotation: 'f'{self.annotation_handle}')

            if is_render_depth:
                self.depth_handle = '{}.{}.{}.depth'.format(pid, '', name)
                self.depth_shmem = mmap.mmap(0, buffer_size, self.depth_handle)
                self.logger.debug('Camera - Bound shared memory for depth: 'f'{self.depth_handle}')

        # Create and initialise the camera in the simulation.
        bng.open_auto_camera(name, vehicle, requested_update_time, update_priority, self.render_size, field_of_view, near_far_planes, pos, dir, up,
            is_using_shared_memory, self.colour_handle, buffer_size, self.annotation_handle, buffer_size, self.depth_handle, buffer_size, is_render_colours, 
            is_render_annotations, is_render_depth, use_instance_annotations, is_visualised, is_static, is_snapping_desired, is_force_inside_triangle)
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

        width = self.render_size[0]
        height = self.render_size[1]

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

        if self.is_render_depth:
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
        if self.is_using_shared_memory:
            if self.is_render_colours:
                self.logger.debug('Camera - Unbinding shared memory for colour: 'f'{self.colour_handle}')
                self.colour_shmem.close()

            if self.is_render_annotations:
                self.logger.debug('Camera - Unbinding shared memory for annotation: 'f'{self.annotation_handle}')
                self.annotation_shmem.close()

            if self.is_render_depth:
                self.logger.debug('Camera - Unbinding shared memory for depth: 'f'{self.depth_handle}')
                self.depth_shmem.close()

        # Remove this sensor from the simulation.
        self.bng.close_auto_camera(self.name, self.use_instance_annotations)
        self.logger.debug('Camera - sensor removed: 'f'{self.name}')

    def poll(self):
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            (dict): The processed images. 
        """

        # Send and receive a request for readings data from this sensor.
        raw_readings = self.bng.poll_auto_camera(self.name, self.is_using_shared_memory, self.use_instance_annotations)['data']

        self.logger.debug('Camera - raw sensor readings received from simulation: 'f'{self.name}')

        # Decode the raw sensor readings into image data. This is handled differently, depending on whether shared memory is used or not.
        width = self.render_size[0]
        height = self.render_size[1]
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
                    images['colour'] = ImageOps.flip(Image.fromarray(colour_d))
                else:
                    self.logger.error('Camera - Colour buffer failed to render. Check that you are not running on low settings.')

            if self.is_render_annotations:
                if 'annotation' in raw_readings.keys():
                    self.annotation_shmem.seek(0)
                    annotate_d = self.annotation_shmem.read(buffer_size)
                    annotate_d = np.frombuffer(annotate_d, dtype=np.uint8)
                    annotate_d = annotate_d.reshape(height, width, 4)
                    images['annotation'] = ImageOps.flip(Image.fromarray(annotate_d))
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
                    images['depth'] = ImageOps.flip(Image.fromarray(depth_values))
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
        return [ table['x'], table['y'], table['z'] ]

    def get_direction(self):
        """
        Gets the current direction vector of this sensor.

        Returns:
            (list): The sensor direction.
        """
        table = self.bng.get_camera_sensor_direction(self.name)['data']
        return [ table['x'], table['y'], table['z'] ]

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