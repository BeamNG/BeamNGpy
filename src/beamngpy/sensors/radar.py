from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

import numpy as np
import struct

from beamngpy.logging import LOGGER_ID, BNGError
from beamngpy.types import Float2, Float3, Int2, StrDict

from .communication_utils import send_sensor_request, set_sensor

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

__all__ = ['Radar']


class Radar:
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
        size: (X, Y) The resolution of the sensor (the size of the depth buffer image in the distance measurement computation).
        field_of_view_y: The sensor vertical field of view parameters.
        near_far_planes: (X, Y) The sensor near and far plane distances.
        range_roundness: the general roudness of the RADAR sensor range-shape. Can be negative.
        range_cutoff_sensitivity: a cutoff sensitivity parameter for the RADAR sensor range-shape.
        range_shape: the shape of the RADAR sensor range-shape in [0, 1], from conical to circular.
        range_focus: the focus parameter for the RADAR sensor range-shape.
        range_min_cutoff: the minimum cut-off distance for the RADAR sensor range-shape. Nothing closer than this will be detected.
        range_direct_max_cutoff: the maximum cut-off distance for the RADAR sensor range-shape. This parameter is a hard cutoff - nothing
            further than this will be detected, although other parameters can also control the max distance.
        is_visualised: Whether or not to render the RADAR sensor points in the simulator.
        is_static: A flag which indicates whether this sensor should be static (fixed position), or attached to a vehicle.
        is_snapping_desired: A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle (not used for static sensors).
        is_force_inside_triangle: A flag which indicates if the sensor should be forced inside the nearest vehicle triangle (not used for static sensors).
    """

    def __init__(self, name: str, bng: BeamNGpy, vehicle: Vehicle | None = None, requested_update_time: float = 0.1,
                 update_priority: float = 0.0, pos: Float3 = (0, 0, 1.7),
                 dir: Float3 = (0, -1, 0), up: Float3 = (0, 0, 1), resolution: Int2 = (200, 200),
                 field_of_view_y: float = 70, near_far_planes: Float2 = (0.1, 150.0),
                 range_roundess: float = -2.0, range_cutoff_sensitivity: float = 0.0, range_shape: float = 0.23,
                 range_focus: float = 0.12, range_min_cutoff: float = 0.5, range_direct_max_cutoff: float = 150.0,
                 is_visualised: bool = True, is_static: bool = False, is_snapping_desired: bool = False, is_force_inside_triangle: bool = False):
        self.logger = getLogger(f'{LOGGER_ID}.RADAR')
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name

        # Create and initialise this sensor in the simulation.
        self._open_radar(
            name, vehicle, requested_update_time, update_priority, pos, dir, up, resolution, field_of_view_y,
            near_far_planes, range_roundess, range_cutoff_sensitivity, range_shape, range_focus, range_min_cutoff,
            range_direct_max_cutoff, is_visualised, is_static, is_snapping_desired, is_force_inside_triangle)
        self.logger.debug('RADAR - sensor created: 'f'{self.name}')

    def _send_sensor_request(self, type: str, ack: str | None = None, **kwargs):
        if not self.bng.connection:
            raise BNGError('The simulator is not connected!')
        return send_sensor_request(self.bng.connection, type, ack, **kwargs)

    def _set_sensor(self, type: str, ack: str | None = None, **kwargs):
        if not self.bng.connection:
            raise BNGError('The simulator is not connected!')
        return set_sensor(self.bng.connection, type, ack, **kwargs)

    def _decode_binary_string(self, binary):
        # Convert the given binary string into an 1D array of floats.
        floats = np.zeros(int(len(binary) / 4))
        ctr = 0
        for i in range(0, int(len(binary)), 4):
            floats[ctr] = struct.unpack('f', binary[i:i + 4])[0]
            ctr = ctr + 1

        # Re-format the float array into a 6D point cloud of raw RADAR data.
        decoded_data = []
        for i in range(0, int(len(floats) / 6), 6):
            decoded_data.append([floats[i], floats[i + 1], floats[i + 2], floats[i + 3], floats[i + 4], floats[i + 5]])

        return decoded_data

    def remove(self):
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_radar()
        self.logger.debug('RADAR - sensor removed: 'f'{self.name}')

    def poll(self):
        """
        Gets the most-recent raw readings for this RADAR sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A 6D point cloud of raw RADAR data, where each entry is (range, doppler velocity, azimuth angle, elevation angle, radar cross section, signal to noise ratio).
        """
        # Send and receive a request for readings data from this sensor.
        binary = self._send_sensor_request('PollRadar', ack='PolledRadar', name=self.name)['data']

        # Convert the binary string into an array of floats.
        radar_data = self._decode_binary_string(binary)

        self.logger.debug('RADAR - sensor readings received from simulation: 'f'{self.name}')

        return radar_data

    def send_ad_hoc_poll_request(self) -> int:
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            A unique Id number for the ad-hoc request.
        """
        self.logger.debug('RADAR - ad-hoc polling request sent: 'f'{self.name}')
        return int(self._send_sensor_request('SendAdHocRequestRadar', ack='CompletedSendAdHocRequestRadar', name=self.name)['data'])

    def is_ad_hoc_poll_request_ready(self, request_id: int) -> bool:
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug('RADAR - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self._send_sensor_request('IsAdHocPollRequestReadyRadar', ack='CompletedIsAdHocPollRequestReadyRadar', requestId=request_id)['data']

    def collect_ad_hoc_poll_request(self, request_id: int):
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            The readings data.
        """
        binary = self._send_sensor_request('CollectAdHocPollRequestRadar', ack='CompletedCollectAdHocPollRequestRadar', requestId=request_id)['data']['radarData']

        radar_data = self._decode_binary_string(binary)

        self.logger.debug('RADAR - ad-hoc polling request returned and processed: 'f'{self.name}')

        return radar_data

    def get_requested_update_time(self) -> float:
        """
        Gets the current 'requested update time' value for this sensor.

        Returns:
            (float): The requested update time.
        """
        return self._send_sensor_request('GetRadarRequestedUpdateTime', ack='CompletedGetRadarRequestedUpdateTime', name=self.name)['data']

    def get_update_priority(self) -> float:
        """
        Gets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, highest to lowest.

        Returns:
            The update priority value.
        """
        return self._send_sensor_request('GetRadarUpdatePriority', ack='CompletedGetRadarUpdatePriority', name=self.name)['data']

    def get_position(self) -> Float3:
        """
        Gets the current world-space position of this sensor.

        Returns:
            The sensor position.
        """
        table = self._send_sensor_request('GetRadarSensorPosition', ack='CompletedGetRadarSensorPosition', name=self.name)['data']
        return (table['x'], table['y'], table['z'])

    def get_direction(self) -> Float3:
        """
        Gets the current direction vector of this sensor.

        Returns:
            The sensor direction.
        """
        table = self._send_sensor_request('GetRadarSensorDirection', ack='CompletedGetRadarSensorDirection', name=self.name)['data']
        return (table['x'], table['y'], table['z'])

    def get_max_pending_requests(self) -> int:
        """
        Gets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Returns:
            The max pending requests value.
        """
        return int(self._send_sensor_request('GetRadarMaxPendingGpuRequests', ack='CompletedGetRadarMaxPendingGpuRequests', name=self.name)['data'])

    def set_requested_update_time(self, requested_update_time: float):
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time: The new requested update time.
        """
        return self._set_sensor('SetRadarRequestedUpdateTime', ack='CompletedSetRadarRequestedUpdateTime', name=self.name, updateTime=requested_update_time)

    def set_update_priority(self, update_priority: float) -> None:
        """
        Sets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, , highest to lowest.

        Args:
            update_priority: The new update priority
        """
        return self._set_sensor('SetRadarUpdatePriority', ack='CompletedSetRadarUpdatePriority', name=self.name, updatePriority=update_priority)

    def set_max_pending_requests(self, max_pending_requests: int) -> None:
        """
        Sets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.

        Args:
            max_pending_requests: The new max pending requests value.
        """
        self._set_sensor('SetRadarMaxPendingGpuRequests', ack='CompletedSetRadarMaxPendingGpuRequests', name=self.name, maxPendingGpuRequests=max_pending_requests)

    def _open_radar(
            self, name: str, vehicle: Vehicle | None, requested_update_time: float, update_priority: float, pos: Float3,
            dir: Float3, up: Float3, size: Int2, field_of_view_y: float, near_far_planes: Float2,
            range_roundness: float, range_cutoff_sensitivity: float, range_shape: float, range_focus: float,
            range_min_cutoff: float, range_direct_max_cutoff: float, is_visualised: bool, is_static: bool,
            is_snapping_desired: bool, is_force_inside_triangle: bool) -> None:
        data: StrDict = dict(type='OpenRadar')
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
        data['isVisualised'] = is_visualised
        data['isStatic'] = is_static
        data['isSnappingDesired'] = is_snapping_desired
        data['isForceInsideTriangle'] = is_force_inside_triangle

        self.bng._send(data).ack('OpenedRadar')
        self.logger.info(f'Opened RADAR sensor: "{name}"')

    def _close_radar(self) -> None:
        data = dict(type='CloseRadar')
        data['name'] = self.name
        self.bng._send(data).ack('ClosedRadar')
        self.logger.info(f'Closed RADAR sensor: "{self.name}"')
