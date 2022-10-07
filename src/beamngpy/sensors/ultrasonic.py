"""
An interactive, automated ultrasonic sensor, which produces regular distance measurements, ready for further processing.
This sensor can be attached to a vehicle, or can be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.
"""

from logging import DEBUG, getLogger

from beamngpy.beamngcommon import LOGGER_ID, ack

from .utils import _send_sensor_request, _set_sensor


class Ultrasonic:
    def __init__(
            self, name, bng, vehicle=None, requested_update_time=0.1, update_priority=0.0, pos=(0, 0, 1.7),
            dir=(0, -1, 0),
            up=(0, 0, 1),
            resolution=(200, 200),
            field_of_view_y=5.7, near_far_planes=(0.1, 5.1),
            range_roundess=-1.15, range_cutoff_sensitivity=0.0, range_shape=0.3, range_focus=0.376,
            range_min_cutoff=0.1, range_direct_max_cutoff=5.0, sensitivity=3.0, fixed_window_size=10,
            is_visualised=True, is_static=False, is_snapping_desired=False, is_force_inside_triangle=False):
        """
        Creates an ultrasonic sensor.
        Args:
            name (str): A unique name for this ultrasonic sensor.
            bng (BeamNGpy): The BeamNGpy instance, with which to communicate to the simulation.
            vehicle (Vehicle class): The vehicle to which this sensor should be attached, if any.
            requested_update_time (float): The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
            update_priority (float): The priority which the sensor should ask for new readings. lowest -> 0, highest -> 1.
            pos (tuple): (X, Y, Z) Coordinate triplet specifying the position of the sensor, in world space.
            dir (tuple): (X, Y, Z) Coordinate triplet specifying the forward direction of the sensor.
            up (tuple): (X, Y, Z) Coordinate triplet specifying the up direction of the sensor.
            size (tuple): (X, Y) The resolution of the sensor (the size of the depth buffer image in the distance measurement computation).
            field_of_view_y (float): The sensor vertical field of view parameters.
            near_far_planes (tuple): (X, Y) The sensor near and far plane distances.
            range_roundness (float): the general roudness of the ultrasonic sensor range-shape. Can be negative.
            range_cutoff_sensitivity (float): a cutoff sensitivity parameter for the ultrasonic sensor range-shape.
            range_shape (float): the shape of the ultrasonic sensor range-shape in [0, 1], from conical to circular.
            range_focus (float): the focus parameter for the ultrasonic sensor range-shape.
            range_min_cutoff (float): the minimum cut-off distance for the ultrasonic sensor range-shape. Nothing closer than this will be detected.
            range_direct_max_cutoff (float): the maximum cut-off distance for the ultrasonic sensor range-shape. This parameter is a hard cutoff - nothing
                further than this will be detected, although other parameters can also control the max distance.
            sensitivity (float): an ultrasonic sensor sensitivity parameter.
            fixed_window_size (float): an ultrasonic sensor sensitivity parameter.
            is_visualised (bool): Whether or not to render the ultrasonic sensor points in the simulator.
            is_static (bool): A flag which indicates whether this sensor should be static (fixed position), or attached to a vehicle.
            is_snapping_desired (bool): A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle (not used for static sensors).
            is_force_inside_triangle (bool): A flag which indicates if the sensor should be forced inside the nearest vehicle triangle (not used for static sensors).
        """
        self.logger = getLogger(f'{LOGGER_ID}.Ultrasonic')
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name

        # Create and initialise this sensor in the simulation.
        self._open_ultrasonic(
            name, vehicle, requested_update_time, update_priority, pos, dir, up, resolution, field_of_view_y,
            near_far_planes, range_roundess, range_cutoff_sensitivity, range_shape, range_focus, range_min_cutoff,
            range_direct_max_cutoff, sensitivity, fixed_window_size, is_visualised, is_static, is_snapping_desired,
            is_force_inside_triangle)
        self.logger.debug('Ultrasonic - sensor created: 'f'{self.name}')

    def _send_sensor_request(self, type, ack=None, **kwargs):
        return _send_sensor_request(self.bng.connection, type, ack, **kwargs)

    def _set_sensor(self, type, **kwargs):
        return _set_sensor(self.bng.connection, type, **kwargs)

    def remove(self):
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_ultrasonic()
        self.logger.debug('Ultrasonic - sensor removed: 'f'{self.name}')

    def poll(self):
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.
        Returns:
            (dict): A dictionary containing the distance measurement and the window (min and mix values) in which it was computed.
        """
        # Send and receive a request for readings data from this sensor.
        distance_measurement = self._send_sensor_request(
            'PollUltrasonic', ack='PolledUltrasonic', name=self.name)['data']
        self.logger.debug('Ultrasonic - sensor readings received from simulation: 'f'{self.name}')

        return distance_measurement

    def send_ad_hoc_poll_request(self):
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.
        Returns:
            (int): A unique Id number for the ad-hoc request.
        """
        self.logger.debug('Ultrasonic - ad-hoc polling request sent: 'f'{self.name}')
        return self._send_sensor_request(
            'SendAdHocRequestUltrasonic', ack='CompletedSendAdHocRequestUltrasonic', name=self.name)['data']

    def is_ad_hoc_poll_request_ready(self, request_id):
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.
        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.
        Returns:
            (bool): A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug('Ultrasonic - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self._send_sensor_request('IsAdHocPollRequestReadyUltrasonic',
                                         ack='CompletedIsAdHocPollRequestReadyUltrasonic', requestId=request_id)

    def collect_ad_hoc_poll_request(self, request_id):
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.
        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.
        Returns:
            (dict): The readings data.
        """
        readings = self._send_sensor_request('CollectAdHocPollRequestUltrasonic',
                                             ack='CompletedCollectAdHocPollRequestUltrasonic', requestId=request_id)['data']
        self.logger.debug('Ultrasonic - ad-hoc polling request returned and processed: 'f'{self.name}')

        return readings

    def get_requested_update_time(self):
        """
        Gets the current 'requested update time' value for this sensor.
        Returns:
            (float): The requested update time.
        """
        return self._send_sensor_request('GetUltrasonicRequestedUpdateTime',
                                         ack='CompletedGetUltrasonicRequestedUpdateTime', name=self.name)['data']

    def get_update_priority(self):
        """
        Gets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, highest to lowest.
        Returns:
            (float): The update priority value.
        """
        return self._send_sensor_request(
            'GetUltrasonicUpdatePriority', ack='CompletedGetUltrasonicUpdatePriority', name=self.name)['data']

    def get_position(self):
        """
        Gets the current world-space position of this sensor.
        Returns:
            (list): The sensor position.
        """
        table = self._send_sensor_request('GetUltrasonicSensorPosition',
                                          ack='CompletedGetUltrasonicSensorPosition', name=self.name)['data']
        return [table['x'], table['y'], table['z']]

    def get_direction(self):
        """
        Gets the current direction vector of this sensor.
        Returns:
            (list): The sensor direction.
        """
        table = self._send_sensor_request('GetUltrasonicSensorDirection',
                                          ack='CompletedGetUltrasonicSensorDirection', name=self.name)['data']
        return [table['x'], table['y'], table['z']]

    def get_max_pending_requests(self):
        """
        Gets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.
        Returns:
            (int): The max pending requests value.
        """
        return self._send_sensor_request('GetUltrasonicMaxPendingGpuRequests',
                                         ack='CompletedGetUltrasonicMaxPendingGpuRequests', name=self.name)['data']

    def get_is_visualised(self):
        """
        Gets a flag which indicates if this ultrasonic sensor is visualised or not.
        Returns:
            (bool): A flag which indicates if this ultrasonic sensor is visualised or not.
        """
        return self._send_sensor_request(
            'GetUltrasonicIsVisualised', ack='CompletedGetUltrasonicIsVisualised', name=self.name)['data']

    @ack('CompletedSetUltrasonicRequestedUpdateTime')
    def set_requested_update_time(self, requested_update_time):
        """
        Sets the current 'requested update time' value for this sensor.
        Args:
            requested_update_time (float): The new requested update time.
        """
        return self._set_sensor('SetUltrasonicRequestedUpdateTime', name=self.name, updateTime=requested_update_time)

    @ack('CompletedSetUltrasonicUpdatePriority')
    def set_update_priority(self, update_priority):
        """
        Sets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, , highest to lowest.
        Args:
            update_priority (float): The new update priority
        """
        return self._set_sensor('SetUltrasonicUpdatePriority', name=self.name, updatePriority=update_priority)

    @ack('CompletedSetUltrasonicMaxPendingGpuRequests')
    def set_max_pending_requests(self, max_pending_requests):
        """
        Sets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.
        Args:
            max_pending_requests (int): The new max pending requests value.
        """
        return self._set_sensor('SetUltrasonicMaxPendingGpuRequests', name=self.name,
                                maxPendingGpuRequests=max_pending_requests)

    @ack('CompletedSetUltrasonicIsVisualised')
    def set_is_visualised(self, is_visualised):
        """
        Sets whether this ultrasonic sensor is to be visualised or not.
        Args:
            is_visualised(bool): A flag which indicates if this ultrasonic sensor is to be visualised or not.
        """
        return self._set_sensor('SetUltrasonicIsVisualised', name=self.name, isVisualised=is_visualised)

    @ack('OpenedUltrasonic')
    def _open_ultrasonic(
            self, name, vehicle, requested_update_time, update_priority, pos, dir, up, size, field_of_view_y,
            near_far_planes, range_roundness, range_cutoff_sensitivity, range_shape, range_focus, range_min_cutoff,
            range_direct_max_cutoff, sensitivity, fixed_window_size, is_visualised, is_static, is_snapping_desired,
            is_force_inside_triangle):
        data = dict(type='OpenUltrasonic')
        data['name'] = name
        data['vid'] = 0
        if vehicle is not None:
            data['vid'] = vehicle.vid
        data['updateTime'] = requested_update_time
        data['priority'] = update_priority
        data['pos'] = pos
        data['dir'] = dir
        data['up'] = up
        data['size'] = size
        data['fovY'] = field_of_view_y
        data['near_far_planes'] = near_far_planes
        data['range_roundness'] = range_roundness
        data['range_cutoff_sensitivity'] = range_cutoff_sensitivity
        data['range_shape'] = range_shape
        data['range_focus'] = range_focus
        data['range_min_cutoff'] = range_min_cutoff
        data['range_direct_max_cutoff'] = range_direct_max_cutoff
        data['sensitivity'] = sensitivity
        data['fixed_window_size'] = fixed_window_size
        data['isVisualised'] = is_visualised
        data['isStatic'] = is_static
        data['isSnappingDesired'] = is_snapping_desired
        data['isForceInsideTriangle'] = is_force_inside_triangle

        resp = self.bng.connection.send(data)
        self.logger.info(f'Opened ultrasonic sensor: "{name}"')
        return resp

    @ack('ClosedUltrasonic')
    def _close_ultrasonic(self):
        data = dict(type='CloseUltrasonic')
        data['name'] = self.name
        resp = self.bng.connection.send(data)
        self.logger.info(f'Closed ultrasonic sensor: "{self.name}"')
        return resp
