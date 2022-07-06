"""
.. module:: lidar
    :platform: Windows
    :synopsis: Module containing the LiDAR sensor.
    :noindex:

.. moduleauthor:: Dave Stark <dstark@beamng.gmbh>

An interactive, managed, automated LiDAR sensor, which produces regular LiDAR point clouds, ready for further processing.
This sensor can be attached to a vehicle, or can be fixed to a position in space.
"""

import os
import mmap
import numpy as np
from logging import DEBUG as DBG_LOG_LEVEL
from logging import getLogger

from .beamngcommon import LOGGER_ID

# The maximum number of LiDAR points which can be used.
MAX_LIDAR_POINTS = 2000000

class Auto_Lidar:

    def __init__(self, name, bng, vehicle=None, requested_update_time=0.1, update_priority=0.0, pos=(0, 0, 1.7), dir=(0, -1, 0), vertical_resolution=64, 
        vertical_angle=26.9, rays_per_second=2200000, frequency=20, horizontal_angle=360, max_distance=120, is_using_shared_memory=True, 
        is_visualised=True, is_annotated=False, is_static=False, is_snapping_desired=False, is_force_inside_triangle=False):

        """
        Creates a LiDAR sensor.

        Args:
            name (str): A unique name for this LiDAR sensor.
            bng (BeamNGpy): The BeamNGpy instance, with which to communicate to the simulation.
            vehicle (Vehicle class): The vehicle to which this sensor should be attached, if any.
            requested_update_time (float): The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
            update_priority (float): The priority which the sensor should ask for new readings. lowest -> 0, highest -> 1. 
            pos (tuple): (X, Y, Z) coordinate triplet specifying the position of the sensor, in world space.
            dir (tuple): (X, Y, Z) coordinate triplet specifying the direction of the sensor.
            vertical_resolution (int): The vertical resolution of this LiDAR sensor.
            vertical_angle (float): The vertical angle of this LiDAR sensor, in degrees.
            rays_per_second (float): The number of LiDAR rays per second which this sensor should emit.
            frequency (float): The frequency of this LiDAR sensor.
            horizontal_angle (float): The horizontal angle of this LiDAR sensor.
            max_distance (float): The maximum distance which this LiDAR sensor will detect, in metres.
            is_using_shared_memory (bool): A flag which indicates if we should use shared memory to send/recieve the sensor readings data.
            is_visualised (bool): A flag which indicates if this LiDAR sensor should appear visualised or not.
            is_annotated (bool): A flag which indicates if this LiDAR sensor should return annotation data instead of distance data.
            is_static (bool): A flag which indicates whether this sensor should be static (fixed position), or attached to a vehicle.
            is_snapping_desired (bool): A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle (not used for static sensors).
            is_force_inside_triangle (bool): A flag which indicates if the sensor should be forced inside the nearest vehicle triangle (not used for static sensors).
        """

        self.logger = getLogger(f'{LOGGER_ID}.Lidar')
        self.logger.setLevel(DBG_LOG_LEVEL)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name

        # Set up the shared memory for this sensor, if requested.
        self.is_using_shared_memory = is_using_shared_memory
        self.shmem_size = MAX_LIDAR_POINTS * 3 * 4
        self.shmem = None
        self.shmem_handle = None
        if is_using_shared_memory:
            pid = os.getpid()
            self.shmem_handle = '{}.{}.{}.Lidar'.format(pid, '', name)
            self.shmem = mmap.mmap(0, self.shmem_size, self.shmem_handle)
            self.logger.debug(f'Lidar - Bound shared memory for point cloud data: {self.shmem_handle}')

        # Create and initialise this sensor in the simulation.
        bng.open_lidar(name, vehicle, is_using_shared_memory, self.shmem, requested_update_time, update_priority, pos, dir, vertical_resolution, 
            vertical_angle, rays_per_second, frequency, horizontal_angle, max_distance, is_visualised, is_annotated, is_static, is_snapping_desired, 
            is_force_inside_triangle)
        self.logger.debug('Lidar - sensor created: 'f'{self.name}')

    def remove(self):
        """
        Removes this sensor from the simulation.
        """

        # Remove the shared memory binding being used by this sensor, if applicable.
        if self.is_using_shared_memory:
            self.logger.debug('Lidar - Unbinding shared memory: 'f'{self.shmem_handle}')
            self.shmem.close()

        # Remove this sensor from the simulation.
        self.bng.close_lidar(self.name)
        self.logger.debug('Lidar - sensor removed: 'f'{self.name}')

    def poll(self):
        """
        Gets the most-recent readings for this sensor.

        Returns:
            (np array): The LiDAR point cloud data.
        """

        # Send and receive a request for readings data from this sensor.
        raw_readings = self.bng.poll_lidar(self.name, self.is_using_shared_memory)['data']
        self.logger.debug('Lidar - raw sensor readings received from simulation: 'f'{self.name}')

        # Get the LiDAR point cloud data and put it in an array.
        point_cloud_data = None
        if self.is_using_shared_memory:
            self.shmem.seek(0)
            point_cloud_data = self.shmem.read(self.shmem_size)
            point_cloud_data = np.frombuffer(point_cloud_data, dtype=np.float32)
            self.logger.debug('Lidar - point cloud data read from shared memory: 'f'{self.name}')
        else:
            point_cloud_data = np.array(raw_readings, dtype=np.float32)
            self.logger.debug('Lidar - point cloud data read from socket: 'f'{self.name}')

        return point_cloud_data

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
