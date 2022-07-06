"""
.. module:: camera
    :platform: Windows
    :synopsis: Module containing the camera sensor.
    :noindex:

.. moduleauthor:: Dave Stark <dstark@beamng.gmbh>

An interactive, managed, automated camera sensor, which can produce regular colour images, depth images, or annotation images.
This sensor can be attached to a vehicle, or can be fixed to a position in space.
"""

import os
import mmap
import math
import base64
import numpy as np
from PIL import Image, ImageOps
from logging import DEBUG as DBG_LOG_LEVEL
from logging import getLogger

from .beamngcommon import LOGGER_ID

class Auto_Camera:

    def __init__(self, name, bng, vehicle=None, requested_update_time=0.1, update_priority=0.0, pos=(0, 0, 3), dir=(0, -1, 0), render_size=(200, 200), 
        field_of_view=(10, 10), near_far_planes=(0.05, 100.0), is_using_shared_memory=True, is_render_colours=True, is_render_annotations=True, 
        is_render_instance=True, is_render_depth=True, is_depth_inverted=False, is_static=False, is_snapping_desired=False, is_force_inside_triangle=False):

        """
        Creates a camera sensor.

        Args:
            name (str): A unique name for this camera sensor.
            bng (BeamNGpy): The BeamNGpy instance, with which to communicate to the simulation.
            vehicle (Vehicle class): The vehicle to which this sensor should be attached, if any.
            requested_update_time (float): The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
            update_priority (float): The priority which the sensor should ask for new readings. lowest -> 0, highest -> 1. 
            pos (tuple): (X, Y, Z) Coordinate triplet specifying the position of the sensor, in world space.
            dir (tuple): (X, Y, Z) Coordinate triplet specifying the direction of the sensor.
            render_size (tuple): (X, Y) The resolution of the sensor images.
            field_of_view (tuple): (X, Y) The sensor field of view parameters.
            near_far_planes (tuple): (X, Y) The sensor near and far plane distances.
            is_using_shared_memory (bool): A flag which indicates if we should use shared memory to send/recieve the sensor readings data.
            is_render_colours (bool): A flag which indicates if this sensor should render colour data.
            is_render_annotations (bool): A flag which indicates if this sensor should render annotation data.
            is_render_instance (bool): A flag which indicates if this sensor should render instance annotation data.
            is_render_depth (bool): A flag which indicates if this sensor should render depth data.
            is_depth_inverted (bool): A flag which indicates if the depth values should be shown white->black or black->white, as distance increases.
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
            buffer_size = render_size[0] * render_size[1] * 4
            if is_render_colours:
                self.colour_handle = '{}.{}.{}.colour'
                self.colour_handle = self.colour_handle.format(pid, '', name)
                self.colour_shmem = mmap.mmap(0, buffer_size, self.colour_handle)
                self.logger.debug('Camera - Bound shared memory for colour: 'f'{self.colour_handle}')

            if is_render_annotations:
                self.annotation_handle = '{}.{}.{}.annotate'
                self.annotation_handle = self.annotation_handle.format(pid, '', name)
                self.annotation_shmem = mmap.mmap(0, buffer_size, self.annotation_handle)
                self.logger.debug('Camera - Bound shared memory for annotation: 'f'{self.annotation_handle}')

            if is_render_instance:
                self.instance_handle = '{}.{}.{}.instance'
                self.instance_handle = self.instance_handle.format(pid, '', name)
                self.instance_shmem = mmap.mmap(0, buffer_size, self.instance_handle)
                self.logger.debug('Camera - Bound shared memory for instance: 'f'{self.instance_handle}')

            if is_render_depth:
                self.depth_handle = '{}.{}.{}.depth'
                self.depth_handle = self.depth_handle.format(pid, '', name)
                self.depth_shmem = mmap.mmap(0, buffer_size, self.depth_handle)
                self.logger.debug('Auto_Camera - Bound shared memory for depth: 'f'{self.depth_handle}')

        # Create and initialise the camera in the simulation.
        bng.open_auto_camera(name, vehicle, requested_update_time, update_priority, self.render_size, field_of_view, near_far_planes, pos, dir, 
            is_using_shared_memory, self.colour_handle, buffer_size, self.annotation_handle, buffer_size, self.depth_handle, buffer_size, is_render_colours, 
            is_render_annotations, is_render_instance, is_render_depth, is_static, is_snapping_desired, is_force_inside_triangle)
        self.logger.debug('Camera - sensor created: 'f'{self.name}')

    def _decode_base64_to_image(self, raw_data, width, height, channels, data_type):
        """
        Decodes and processes raw camera data from a base64 buffer into image format.

        Args:
            raw_data (array): The 1D buffer which is to be decoded and processed.
            width (int): The width of the image to be rendered.
            height (int): The height of the image to be rendered.
            channels(int): The number of channels in the data, here either 4 for RGBA or 1 for depth data. 
            data_type: The type of data, eg np.uint8, np.float32.

        Returns:
            (PIL image object): The decoded and processed image. 
        """

        # Convert from base64 to a numpy array.
        decoded = base64.b64decode(raw_data)
        decoded = np.frombuffer(decoded, dtype=data_type)

        # Re-shape the array, based on the number of channels present in the data.
        if channels > 1:
            decoded = decoded.reshape(height, width, channels)
        else:
            decoded = decoded.reshape(height, width)

        # Convert to image format.
        return ImageOps.flip(Image.fromarray(decoded))

    def _depth_buffer_processing(self, raw_depth_values):
        """
        Converts raw depth buffer data to visually clear intensity values in the range [0, 255].
        We process the data so that small changes in distance are better shown, rather than just using linear interpolation.

        Args:
            raw_depth_values (array): The raw 1D buffer of depth values.

        Returns:
            (array): The processed intensity values in the range [0, 255].
        """

        # Sort the depth values, and cache the sorting map.
        s_data = sorted(raw_depth_values)
        sort_index = np.argsort(raw_depth_values)

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

            if self.is_render_instance:
                self.logger.debug('Camera - Unbinding shared memory for instance: 'f'{self.instance_handle}')
                self.instance_shmem.close()

            if self.is_render_depth:
                self.logger.debug('Camera - Unbinding shared memory for depth: 'f'{self.depth_handle}')
                self.depth_shmem.close()

        # Remove this sensor from the simulation.
        self.bng.close_auto_camera(self.name)
        self.logger.debug('Camera - sensor removed: 'f'{self.name}')

    def poll(self):
        """
        Gets the most-recent readings for this sensor.

        Returns:
            (dict): The processed images. 
        """

        # Send and receive a request for readings data from this sensor.
        raw_readings = self.bng.poll_auto_camera(self.name, self.is_using_shared_memory)['data']
        self.logger.debug('Camera - raw sensor readings received from simulation: 'f'{self.name}')

        # Decode the raw sensor readings into image data. This is handled differently, depending on whether shared memory is used or not.
        width = self.render_size[0]
        height = self.render_size[1]
        buffer_size = width * height * 4
        processed_readings = dict(type='Camera')
        if self.is_using_shared_memory:

            # CASE 1: We are using shared memory.
            if self.is_render_colours:
                if 'colourRGB8' in raw_readings.keys():
                    self.colour_shmem.seek(0)
                    colour_d = self.colour_shmem.read(buffer_size)
                    colour_d = np.frombuffer(colour_d, dtype=np.uint8)
                    colour_d = colour_d.reshape(height, width, 4)
                    processed_readings['colour'] = ImageOps.flip(Image.fromarray(colour_d))
                else:
                    self.logger.error('Camera - Colour buffer failed to render. Check that you are not running on low settings.')

            if self.is_render_annotations:
                if 'annotationRGB8' in raw_readings.keys():
                    self.annotation_shmem.seek(0)
                    annotate_d = self.annotation_shmem.read(buffer_size)
                    annotate_d = np.frombuffer(annotate_d, dtype=np.uint8)
                    annotate_d = annotate_d.reshape(height, width, 4)
                    processed_readings['annotation'] = ImageOps.flip(Image.fromarray(annotate_d))
                else:
                    self.logger.error('Camera - Annotation buffer failed to render. Check that you are not running on low settings.')

            if self.is_render_instance:
                if 'instance' in raw_readings.keys():
                    self.instance_shmem.seek(0)
                    instance_d = self.instance_shmem.read(buffer_size)
                    instance_d = np.frombuffer(instance_d, dtype=np.uint8)
                    instance_d = instance_d.reshape(height, width, 4)
                    processed_readings['instance'] = ImageOps.flip(Image.fromarray(instance_d))
                else:
                    self.logger.error('Camera - Instance buffer failed to render. Check that you are not running on low settings.')

            if self.is_render_depth:
                if 'depth32F' in raw_readings.keys():
                    self.depth_shmem.seek(0)
                    depth_values = self.depth_shmem.read(buffer_size)
                    depth_values = np.frombuffer(depth_values, dtype=np.float32)
                    depth_values = self._depth_buffer_processing(depth_values)
                    depth_values = depth_values.reshape(height, width)
                    depth_values = np.uint8(depth_values)
                    processed_readings['depth'] = ImageOps.flip(Image.fromarray(depth_values))
                else:
                    self.logger.error('Camera - Depth buffer failed to render. Check that you are not running on low settings.')

            self.logger.debug('Camera - sensor readings read from shared memory and processed: 'f'{self.name}')
        else:
            
            # CASE 2: We are not using shared memory. The data is coming back across the socket as a buffer encoded in base64.
            if self.is_render_colours:
                processed_readings['colour'] = self._decode_base64_to_image(raw_readings['colourRGB8'], width, height, 4, np.uint8)

            if self.is_render_annotations:
                processed_readings['annotation'] = self._decode_base64_to_image(raw_readings['annotationRGB8'], width, height, 4, np.uint8)

            if self.is_render_instance:
                processed_readings['instance'] = self._decode_base64_to_image(raw_readings['instanceRGB8'], width, height, 4, np.uint8)

            if self.is_render_depth:
                decoded_values = base64.b64decode(raw_readings['depth32F'])
                decoded_values = np.frombuffer(decoded_values, dtype=np.float32)
                processed_values = self._depth_buffer_processing(decoded_values)
                reshaped_data = processed_values.reshape(height, width)
                processed_readings['depth'] = ImageOps.flip(Image.fromarray(reshaped_data))

            self.logger.debug('Camera - raw sensor readings converted from base64 to image format: 'f'{self.name}')
        return processed_readings

    def get_requested_update_time(self):
        """
        Gets the current 'requested update time' value for this sensor.
        """
        pass

    def get_update_priority(self):
        """
        Gets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, highest to lowest.
        """
        pass

    def get_position(self):
        """
        Gets the current world-space position of this sensor.
        """
        pass

    def get_direction(self):
        """
        Gets the current direction vector of this sensor.
        """
        pass

    def get_max_pending_requests(self):
        """
        Gets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.
        """
        pass

    def set_requested_update_time(self):
        """
        Sets the current 'requested update time' value for this sensor.
        """
        pass

    def set_update_priority(self):
        """
        Sets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, , highest to lowest.
        """
        pass

    def set_max_pending_requests(self):
        """
        Sets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.
        """
        pass
