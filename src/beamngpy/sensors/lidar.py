"""
An interactive, automated LiDAR sensor, which produces regular LiDAR point clouds, ready for further processing.
This sensor can be attached to a vehicle, or can be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.
"""

import mmap
import os
import struct
from logging import DEBUG, getLogger

import numpy as np
from beamngpy.beamngcommon import LOGGER_ID, ack

from .utils import _send_sensor_request, _set_sensor

# The maximum number of LiDAR points which can be used.
# TODO: Make this more efficient by instead computing the number of LiDAR points based on the sensor parameter values.
MAX_LIDAR_POINTS = 2000000


class Lidar:
    def __init__(
            self, name, bng, vehicle=None, requested_update_time=0.1, update_priority=0.0,
            pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1),
            vertical_resolution=64, vertical_angle=26.9, rays_per_second=2200000, frequency=20, horizontal_angle=360,
            max_distance=120, is_using_shared_memory=True, is_visualised=True, is_annotated=False, is_static=False,
            is_snapping_desired=False, is_force_inside_triangle=False):
        """
        Creates a LiDAR sensor.

        Args:
            name (str): A unique name for this LiDAR sensor.
            bng (BeamNGpy): The BeamNGpy instance, with which to communicate to the simulation.
            vehicle (Vehicle class): The vehicle to which this sensor should be attached, if any.
            requested_update_time (float): The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
            update_priority (float): The priority which the sensor should ask for new readings. lowest -> 0, highest -> 1.
            pos (tuple): (X, Y, Z) coordinate triplet specifying the position of the sensor, in world space.
            dir (tuple): (X, Y, Z) Coordinate triplet specifying the forward direction of the sensor.
            up (tuple): (X, Y, Z) Coordinate triplet specifying the up direction of the sensor.
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
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name

        # Set up the shared memory for this sensor, if requested.
        self.is_using_shared_memory = is_using_shared_memory
        self.point_cloud_shmem_size = MAX_LIDAR_POINTS * 3 * 4
        self.point_cloud_shmem = None
        self.point_cloud_shmem_handle = None
        self.colour_shmem_size = MAX_LIDAR_POINTS * 4
        self.colour_shmem = None
        self.colour_shmem_handle = None
        if is_using_shared_memory:
            pid = os.getpid()
            self.point_cloud_shmem_handle = '{}.{}.{}.PointCloud'.format(pid, '', name)
            self.point_cloud_shmem = mmap.mmap(0, self.point_cloud_shmem_size, self.point_cloud_shmem_handle)
            self.logger.debug(f'Lidar - Bound shared memory for point cloud data: {self.point_cloud_shmem_handle}')

            self.colour_shmem_handle = '{}.{}.{}.Colour'.format(pid, '', name)
            self.colour_shmem = mmap.mmap(0, self.colour_shmem_size, self.colour_shmem_handle)
            self.logger.debug(f'Lidar - Bound shared memory for colour data: {self.colour_shmem_handle}')

        # Create and initialise this sensor in the simulation.
        self._open_lidar(
            name, vehicle, is_using_shared_memory, self.point_cloud_shmem_handle, self.point_cloud_shmem_size, self.
            colour_shmem_handle, self.colour_shmem_size, requested_update_time, update_priority, pos, dir, up,
            vertical_resolution, vertical_angle, rays_per_second, frequency, horizontal_angle, max_distance,
            is_visualised, is_annotated, is_static, is_snapping_desired, is_force_inside_triangle)
        self.logger.debug('Lidar - sensor created: 'f'{self.name}')

    def _send_sensor_request(self, type, **kwargs):
        return _send_sensor_request(self.bng.connection, type, **kwargs)

    def _set_sensor(self, type, **kwargs):
        return _set_sensor(self.bng.connection, type, **kwargs)

    def _convert_binary_to_array(self, binary):
        """
        Converts the binary string data from the simulator, which contains the point cloud and colour data, into arrays.

        Args:
            binary (binary string): The raw readings data, as a binary string.
        Returns:
            (dict): A dictionary containing the point cloud and colour data.
        """
        processed_readings = dict(type='Lidar')

        # Format the point cloud data.
        floats = np.zeros(int(len(binary['pointCloud']) / 4))
        ctr = 0
        # Convert the binary string to a float array.
        for i in range(0, int(len(binary['pointCloud'])), 4):
            floats[ctr] = struct.unpack('f', binary['pointCloud'][i:i + 4])[0]
            ctr = ctr + 1
        points = []
        # Convert the floats to points by collecting each triplet.
        for i in range(0, int(len(floats) / 3), 3):
            points.append([floats[i], floats[i + 1], floats[i + 2]])
        processed_readings['pointCloud'] = points

        # Format the corresponding colour data.
        colour = []
        for i in range(len(binary['colours'])):
            colour.append(np.uint8(binary['colours'][i]))
        processed_readings['colours'] = colour

        return processed_readings

    def remove(self):
        """
        Removes this sensor from the simulation.
        """
        # Remove the shared memory binding being used by this sensor, if applicable.
        if self.is_using_shared_memory:
            self.logger.debug('Lidar - Unbinding shared memory: 'f'{self.point_cloud_shmem_handle}')
            self.point_cloud_shmem.close()
            self.logger.debug('Lidar - Unbinding shared memory: 'f'{self.colour_shmem_handle}')
            self.colour_shmem.close()

        # Remove this sensor from the simulation.
        self._close_lidar()
        self.logger.debug('Lidar - sensor removed: 'f'{self.name}')

    def poll(self):
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            (np.array x 2): The LiDAR point cloud and colour data.
        """
        processed_readings = dict(type='Lidar')

        # Get the LiDAR point cloud and colour data, and format it before returning it.
        point_cloud_data = None
        colour_data = None
        if self.is_using_shared_memory:
            data_sizes = self._send_sensor_request('PollLidar', name=self.name,
                                                   isUsingSharedMemory=self.is_using_shared_memory)['data']
            self.point_cloud_shmem.seek(0)
            point_cloud_data = self.point_cloud_shmem.read(self.point_cloud_shmem_size)
            point_cloud_data = np.frombuffer(point_cloud_data, dtype=np.float32)
            processed_readings['pointCloud'] = point_cloud_data
            self.logger.debug('Lidar - point cloud data read from shared memory: 'f'{self.name}')

            self.colour_shmem.seek(0)
            colour_data = self.colour_shmem.read(self.colour_shmem_size)
            colour_data = np.frombuffer(colour_data, dtype=np.uint8)
            processed_readings['colours'] = colour_data
            self.logger.debug('Lidar - colour data read from shared memory: 'f'{self.name}')
        else:
            binary = self._send_sensor_request('PollLidar', name=self.name,
                                               isUsingSharedMemory=self.is_using_shared_memory)['data']
            self.logger.debug('Lidar - LiDAR data read from socket: 'f'{self.name}')
            processed_readings = self._convert_binary_to_array(binary)

        return processed_readings

    def send_ad_hoc_poll_request(self):
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            (int): A unique Id number for the ad-hoc request.
        """
        self.logger.debug('Lidar - ad-hoc polling request sent: 'f'{self.name}')
        return self._send_sensor_request('SendAdHocRequestLidar', name=self.name)['data']

    def is_ad_hoc_poll_request_ready(self, request_id):
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (bool): A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug('Lidar - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self._send_sensor_request('IsAdHocPollRequestReadyLidar', requestId=request_id)

    def collect_ad_hoc_poll_request(self, request_id):
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.
        Returns:
            (dict): A dictionary containing the LiDAR point cloud and colour data.
        """
        # Get the binary string data from the simulator.
        binary = self._send_sensor_request('CollectAdHocPollRequestLidar', requestId=request_id)['data']

        self.logger.debug('Lidar - LiDAR data read from socket: 'f'{self.name}')
        return self._convert_binary_to_array(binary)

    def get_requested_update_time(self):
        """
        Gets the current 'requested update time' value for this sensor.

        Returns:
            (float): The requested update time.
        """
        return self._send_sensor_request('GetLidarRequestedUpdateTime', name=self.name)['data']

    def get_update_priority(self):
        """
        Gets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, highest to lowest.

        Returns:
            (float): The update priority value.
        """
        return self._send_sensor_request('GetLidarUpdatePriority', name=self.name)['data']

    def get_position(self):
        """
        Gets the current world-space position of this sensor.

        Returns:
            (list): The sensor position.
        """
        table = self._send_sensor_request('GetLidarSensorPosition', name=self.name)['data']
        return [table['x'], table['y'], table['z']]

    def get_direction(self):
        """
        Gets the current direction vector of this sensor.

        Returns:
            (list): The sensor direction.
        """
        table = self._send_sensor_request('GetLidarSensorDirection', name=self.name)['data']
        return [table['x'], table['y'], table['z']]

    def get_max_pending_requests(self):
        """
        Gets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Returns:
            (int): The max pending requests value.
        """
        return self._send_sensor_request('GetLidarMaxPendingGpuRequests', name=self.name)['data']

    def get_is_visualised(self):
        """
        Gets a flag which indicates if this LiDAR sensor is visualised or not.

        Returns:
            (bool): A flag which indicates if this LiDAR sensor is visualised or not.
        """
        return self._send_sensor_request('GetLidarIsVisualised', name=self.name)['data']

    def get_is_annotated(self):
        """
        Gets a flag which indicates if this LiDAR sensor is annotated or not.

        Returns:
            (bool): A flag which indicates if this LiDAR sensor is annotated or not.
        """
        return self._send_sensor_request('GetLidarIsAnnotated', name=self.name)['data']

    @ack('CompletedSetLidarRequestedUpdateTime')
    def set_requested_update_time(self, requested_update_time):
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            update_priority (float): The new requested update time.
        """
        return self._set_sensor('SetLidarRequestedUpdateTime', name=self.name, updateTime=requested_update_time)

    @ack('CompletedSetLidarUpdatePriority')
    def set_update_priority(self, update_priority):
        """
        Sets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, , highest to lowest.

        Args:
            update_priority (float): The new update priority value.
        """
        return self._set_sensor('SetLidarUpdatePriority', name=self.name, updatePriority=update_priority)

    @ack('CompletedSetLidarMaxPendingGpuRequests')
    def set_max_pending_requests(self, max_pending_requests):
        """
        Sets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Args:
            update_priority (int): The new max pending requests value.
        """
        return self._set_sensor('SetLidarMaxPendingGpuRequests', name=self.name,
                                maxPendingGpuRequests=max_pending_requests)

    @ack('CompletedSetLidarIsVisualised')
    def set_is_visualised(self, is_visualised):
        """
        Sets whether this LiDAR sensor is to be visualised or not.

        Args:
            is_visualised(bool): A flag which indicates if this LiDAR sensor is to be visualised or not.
        """
        return self._set_sensor('SetLidarIsVisualised', name=self.name, isVisualised=is_visualised)

    @ack('CompletedSetLidarIsAnnotated')
    def set_is_annotated(self, is_annotated):
        """
        Sets whether this LiDAR sensor is to be annotated or not. This means it will return annotation data instead of distances.

        Args:
            is_visualised(bool): A flag which indicates if this LiDAR sensor is to be annotated or not.
        """
        return self._set_sensor('SetLidarIsAnnotated', name=self.name, isAnnotated=is_annotated)

    @ack('OpenedLidar')
    def _open_lidar(self, name, vehicle, is_using_shared_memory, point_cloud_shmem_handle, point_cloud_shmem_size,
                    colour_shmem_handle, colour_shmem_size, requested_update_time, update_priority, pos, dir, up,
                    vertical_resolution, vertical_angle, rays_per_second, frequency, horizontal_angle, max_distance,
                    is_visualised, is_annotated, is_static, is_snapping_desired, is_force_inside_triangle):
        data = dict(type='OpenLidar')
        data['vid'] = 0
        if vehicle is not None:
            data['vid'] = vehicle.vid
        data['useSharedMemory'] = is_using_shared_memory
        data['name'] = name
        data['pointCloudShmemHandle'] = point_cloud_shmem_handle
        data['pointCloudShmemSize'] = point_cloud_shmem_size
        data['colourShmemHandle'] = colour_shmem_handle
        data['colourShmemSize'] = colour_shmem_size
        data['updateTime'] = requested_update_time
        data['priority'] = update_priority
        data['pos'] = pos
        data['dir'] = dir
        data['up'] = up
        data['vRes'] = vertical_resolution
        data['vAngle'] = vertical_angle
        data['rps'] = rays_per_second
        data['hz'] = frequency
        data['hAngle'] = horizontal_angle
        data['maxDist'] = max_distance
        data['isVisualised'] = is_visualised
        data['isAnnotated'] = is_annotated
        data['isStatic'] = is_static
        data['isSnappingDesired'] = is_snapping_desired
        data['isForceInsideTriangle'] = is_force_inside_triangle
        resp = self.bng.connection.send(data)
        self.logger.info(f'Opened lidar: "{name}')
        return resp

    @ack('ClosedLidar')
    def _close_lidar(self):
        """
        Closes the Lidar instance of the given name in the simulator.

        Args:
            name (str): The name of the Lidar instance to close.
        """
        data = dict(type='CloseLidar')
        data['name'] = self.name
        resp = self.bng.connection.send(data)
        self.logger.info(f'Closed lidar: "{self.name}"')
        return resp
