"""
An interactive, automated IMU sensor, which produces regular acceleration and gyroscopic measurements in a local coordinate space.
This sensor must be attached to a vehicle; it cannot be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.
We can set this sensor to poll the send data back in two modes:
i) immediate mode: data is sent back as soon as it is available (single readings arrive instantly) - this method is suitable when working with
tightly-coupled systems requiring fast feedback, or
ii) post-processing mode: we can set it to send the data back in bulk on the simulations graphics step - this method is appropriate for the case when the
user wishes simply to post-process the data (such as for plotting graphs etc) and is also more efficient. In this case, the returned data will contain all
the individual samples which were measured in the simulations physics step, so the data is the same as in mode i); it just arrives later, in bulk.
"""

from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Optional

from beamngpy.beamngcommon import LOGGER_ID, BNGError, ack

from .communication_utils import send_sensor_request, set_sensor

if TYPE_CHECKING:
    from ..beamng import BeamNGpy, Vehicle, Float3


class Advanced_IMU:
    def __init__(
            self, name: str, bng: BeamNGpy, vehicle: Vehicle, gfx_update_time=0.1, physics_update_time=0.015,
            pos: Float3 = (0, 0, 1.7), dir: Float3 = (0, -1, 0), up: Float3 = (0, 0, 1),
            window_width: Optional[float] = None, frequency_cutoff: Optional[float] = None, is_send_immediately=False, is_using_gravity=False,
            is_visualised=True, is_snapping_desired=False, is_force_inside_triangle=False):
        """
        Creates an advanced IMU sensor.

        Args:
            name (str): A unique name for this advanced IMU sensor.
            bng (BeamNGpy): The BeamNGpy instance, with which to communicate to the simulation.
            vehicle (Vehicle class): The vehicle to which this sensor should be attached. Note: a vehicle must be provided for the advanced IMU sensor.
            gfx_update_time (float): The gfx-step time which should pass between sensor reading updates to the user, in seconds.
            physics_update_time (float): The physics-step time which should pass between actual sampling the sensor, in seconds.
            pos (tuple): (X, Y, Z) Coordinate triplet specifying the position of the sensor, in world space.
            dir (tuple): (X, Y, Z) Coordinate triplet specifying the forward direction of the sensor.
            up (tuple): (X, Y, Z) Coordinate triplet specifying the up direction of the sensor.
            window_width (float): The width of the window used in smoothing the data, if required.
            frequency_cutoff (float): The filtering cutoff frequency to be used (instead of a window width). of required.
            is_send_immediately (bool): A flag which indicates if the readings should be sent back as soon as available or upon graphics step updates, as bulk.
            is_using_gravity (bool): A flag which indicates whether this sensor should consider acceleration due to gravity in its computations, or not.
            is_visualised (bool): Whether or not to render the ultrasonic sensor points in the simulator.
            is_snapping_desired (bool): A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle.
            is_force_inside_triangle (bool): A flag which indicates if the sensor should be forced inside the nearest vehicle triangle.
        """

        self.logger = getLogger(f'{LOGGER_ID}.Advanced IMU')
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name
        self.vehicle = vehicle
        self.is_send_immediately = is_send_immediately

        # Create and initialise this sensor in the simulation.
        self._open_advanced_IMU(
            name, vehicle, gfx_update_time, physics_update_time, pos, dir, up, window_width, is_send_immediately,
            frequency_cutoff, is_using_gravity, is_visualised, is_snapping_desired, is_force_inside_triangle)

        # Fetch the unique Id number (in the simulator) for this advanced IMU sensor.  We will need this later.
        self.sensorId = int(self._get_advanced_imu_id())

        self.logger.debug('Advanced IMU - sensor created: 'f'{self.name}')

    def _send_sensor_request(self, type: str, ack: Optional[str] = None, **kwargs):
        if not self.bng.connection:
            raise BNGError('The simulator is not connected!')
        return send_sensor_request(self.bng.connection, type, ack, **kwargs)

    def _set_sensor(self, type: str, **kwargs):
        if not self.bng.connection:
            raise BNGError('The simulator is not connected!')
        return set_sensor(self.bng.connection, type, **kwargs)

    def remove(self):
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_advanced_IMU()
        self.logger.debug('Advanced IMU - sensor removed: 'f'{self.name}')

    def poll(self):
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            (dict): A dictionary containing the sensor readings data.
        """

        # Send and receive a request for readings data from this sensor.
        readings_data = []
        if self.is_send_immediately:
            # Get the most-recent single reading from vlua.
            readings_data = self._poll_advanced_IMU_VE()
        else:
            # Get the bulk data from ge lua.
            readings_data = self._poll_advanced_IMU_GE()

        self.logger.debug('Advanced IMU - sensor readings received from simulation: 'f'{self.name}')
        return readings_data

    def send_ad_hoc_poll_request(self):
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            (int): A unique Id number for the ad-hoc request.
        """
        self.logger.debug('Advanced IMU - ad-hoc polling request sent: 'f'{self.name}')
        return self._send_sensor_request('SendAdHocRequestAdvancedIMU', 'CompletedSendAdHocRequestAdvancedIMU',
                                         name=self.name, vid=self.vehicle.vid)['data']

    def is_ad_hoc_poll_request_ready(self, request_id):
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (bool): A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug('Advanced IMU - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self._send_sensor_request('IsAdHocPollRequestReadyAdvancedIMU',
                                         'CompletedIsAdHocPollRequestReadyAdvancedIMU', requestId=request_id)['data']

    def collect_ad_hoc_poll_request(self, request_id):
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (dict): The readings data.
        """
        readings = self._send_sensor_request('CollectAdHocPollRequestAdvancedIMU',
                                             'CompletedCollectAdHocPollRequestAdvancedIMU', requestId=request_id)['data']
        self.logger.debug('Advanced IMU - ad-hoc polling request returned and processed: 'f'{self.name}')
        return readings

    def get_position(self):
        """
        Gets the current world-space position of this sensor.

        Returns:
            (list): The sensor position.
        """
        table = self._send_sensor_request('GetAdvancedIMUSensorPosition',
                                          'CompletedGetAdvancedIMUSensorPosition', name=self.name)['data']
        return [table['x'], table['y'], table['z']]

    def get_direction(self):
        """
        Gets the current direction vector of this sensor.

        Returns:
            (list): The sensor direction.
        """
        table = self._send_sensor_request('GetAdvancedIMUSensorDirection',
                                          'CompletedGetAdvancedIMUSensorDirection', name=self.name)['data']
        return [table['x'], table['y'], table['z']]

    @ack('CompletedSetAdvancedIMURequestedUpdateTime')
    def set_requested_update_time(self, requested_update_time):
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time (float): The new requested update time.
        """
        return self._set_sensor(
            'SetAdvancedIMURequestedUpdateTime', name=self.name, vid=self.vehicle.vid,
            GFXUpdateTime=requested_update_time)

    @ack('CompletedSetAdvancedIMUIsUsingGravity')
    def set_is_using_gravity(self, is_using_gravity):
        """
        Sets whether this sensor is to include gravity in the computation or not.

        Args:
            is_visualised(bool): A flag which indicates if this sensor is to use gravity in the computation or not.
        """
        return self._set_sensor('SetAdvancedIMUIsUsingGravity', name=self.name, vid=self.vehicle.vid,
                                isUsingGravity=is_using_gravity)

    @ack('CompletedSetAdvancedIMUIsVisualised')
    def set_is_visualised(self, is_visualised):
        """
        Sets whether this sensor is to be visualised or not.

        Args:
            is_visualised(bool): A flag which indicates if this sensor is to be visualised or not.
        """
        return self._set_sensor('SetAdvancedIMUIsVisualised', name=self.name, vid=self.vehicle.vid,
                                isVisualised=is_visualised)

    def _get_advanced_imu_id(self):
        return self._send_sensor_request('GetAdvancedImuId', ack='CompletedGetAdvancedImuId', name=self.name)['data']

    @ack('OpenedAdvancedIMU')
    def _open_advanced_IMU(
            self, name: str, vehicle: Vehicle, gfx_update_time, physics_update_time, pos, dir, up, window_width, is_send_immediately,
            frequency_cutoff, is_using_gravity, is_visualised, is_snapping_desired, is_force_inside_triangle):
        data = dict(type='OpenAdvancedIMU')
        data['name'] = name
        data['vid'] = vehicle.vid
        data['GFXUpdateTime'] = gfx_update_time
        data['physicsUpdateTime'] = physics_update_time
        data['pos'] = pos
        data['dir'] = dir
        data['up'] = up
        data['windowWidth'] = window_width
        data['frequencyCutoff'] = frequency_cutoff
        data['isSendImmediately'] = is_send_immediately
        data['isUsingGravity'] = is_using_gravity
        data['isVisualised'] = is_visualised
        data['isSnappingDesired'] = is_snapping_desired
        data['isForceInsideTriangle'] = is_force_inside_triangle
        resp = self.bng.send(data)
        self.logger.info(f'Opened advanced IMU sensor: "{name}"')
        return resp

    @ack('ClosedAdvancedIMU')
    def _close_advanced_IMU(self):
        data = dict(type='CloseAdvancedIMU')
        data['name'] = self.name
        data['vid'] = self.vehicle.vid
        resp = self.bng.send(data)
        self.logger.info(f'Closed advanced IMU sensor: "{self.name}"')
        return resp

    def _poll_advanced_IMU_GE(self):
        return self._send_sensor_request('PollAdvancedImuGE', ack='PolledAdvancedImuGECompleted', name=self.name)['data']

    def _poll_advanced_IMU_VE(self):
        if not self.vehicle.connection:
            raise BNGError('The vehicle is not connected!')
        return send_sensor_request(self.vehicle.connection, 'PollAdvancedImuVE', name=self.name, sensorId=self.sensorId)['data']
