from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any

from beamngpy.logging import LOGGER_ID, BNGError
from beamngpy.types import Float3, StrDict

from .communication_utils import send_sensor_request, set_sensor

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

__all__ = ['GPS']


class GPS:
    """
    GPS Sensor class.
    """

    def __init__(self, name: str, bng: BeamNGpy, vehicle: Vehicle, gfx_update_time: float = 0.0, physics_update_time: float = 0.01, pos: Float3 = (0, 0, 1.7),
        refLon: float = 0.0, refLat: float = 0.0, is_send_immediately: bool = False, is_visualised: bool = True, is_snapping_desired: bool = False,
        is_force_inside_triangle: bool = False):

        self.logger = getLogger(f'{LOGGER_ID}.GPS')
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name
        self.vehicle = vehicle
        self.is_send_immediately = is_send_immediately

        # Create and initialise this sensor in the simulation.
        self._open_GPS(name, vehicle, gfx_update_time, physics_update_time, pos, refLon, refLat, is_send_immediately,
            is_visualised, is_snapping_desired, is_force_inside_triangle)

        # Fetch the unique Id number (in the simulator) for this GPS sensor.  We will need this later.
        self.sensorId = self._get_GPS_id()

        self.logger.debug('GPS - sensor created: 'f'{self.name}')

    def _send_sensor_request(self, type: str, ack: str | None = None, **kwargs: Any) -> StrDict:
        if not self.bng.connection:
            raise BNGError('The simulator is not connected!')
        return send_sensor_request(self.bng.connection, type, ack, **kwargs)

    def _set_sensor(self, type: str, ack: str | None = None, **kwargs: Any) -> None:
        if not self.bng.connection:
            raise BNGError('The simulator is not connected!')
        set_sensor(self.bng.connection, type, **kwargs)

    def remove(self) -> None:
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_GPS()
        self.logger.debug('GPS - sensor removed: 'f'{self.name}')

    def poll(self) -> StrDict:
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary containing the sensor readings data.
        """
        # Send and receive a request for readings data from this sensor.
        readings_data = []
        if self.is_send_immediately:
            # Get the most-recent single reading from vlua.
            readings_data = self._poll_GPS_VE()
        else:
            # Get the bulk data from ge lua.
            readings_data = self._poll_GPS_GE()

        self.logger.debug('GPS - sensor readings received from simulation: 'f'{self.name}')
        return readings_data

    def send_ad_hoc_poll_request(self) -> int:
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            A unique Id number for the ad-hoc request.
        """
        self.logger.debug('GPS - ad-hoc polling request sent: 'f'{self.name}')
        return int(self._send_sensor_request('SendAdHocRequestGPS', 'CompletedSendAdHocRequestGPS', name=self.name, vid=self.vehicle.vid)['data'])

    def is_ad_hoc_poll_request_ready(self, request_id: int) -> bool:
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug('GPS - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self._send_sensor_request('IsAdHocPollRequestReadyGPS', 'CompletedIsAdHocPollRequestReadyGPS', requestId=request_id)['data']

    def collect_ad_hoc_poll_request(self, request_id: int) -> StrDict:
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            The readings data.
        """
        readings = self._send_sensor_request('CollectAdHocPollRequestGPS', 'CompletedCollectAdHocPollRequestGPS', requestId=request_id)['data']
        self.logger.debug('GPS - ad-hoc polling request returned and processed: 'f'{self.name}')
        return readings

    def set_requested_update_time(self, requested_update_time: float) -> None:
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time: The new requested update time.
        """
        self._set_sensor('SetGPSRequestedUpdateTime', ack='CompletedSetGPSRequestedUpdateTime', name=self.name, vid=self.vehicle.vid, GFXUpdateTime=requested_update_time)

    def set_is_visualised(self, is_visualised: bool) -> None:
        """
        Sets whether this sensor is to be visualised or not.

        Args:
            is_visualised: A flag which indicates if this sensor is to be visualised or not.
        """
        self._set_sensor('SetGPSIsVisualised', ack='CompletedSetGPSIsVisualised', name=self.name, vid=self.vehicle.vid, isVisualised=is_visualised)

    def _get_GPS_id(self) -> int:
        return int(
            self._send_sensor_request('GetGPSId', ack='CompletedGetGPSId', name=self.name)['data'])

    def _open_GPS(self, name: str, vehicle: Vehicle, gfx_update_time: float, physics_update_time: float, pos: Float3, refLon: float, refLat: float,
        is_send_immediately: bool, is_visualised: bool, is_snapping_desired: bool, is_force_inside_triangle: bool) -> None:
        data: StrDict = dict(type='OpenGPS')
        data['name'] = name
        data['vid'] = vehicle.vid
        data['GFXUpdateTime'] = gfx_update_time
        data['physicsUpdateTime'] = physics_update_time
        data['pos'] = pos
        data['refLon'] = refLon
        data['refLat'] = refLat
        data['isSendImmediately'] = is_send_immediately
        data['isVisualised'] = is_visualised
        data['isSnappingDesired'] = is_snapping_desired
        data['isForceInsideTriangle'] = is_force_inside_triangle
        self.bng._send(data).ack('OpenedGPS')
        self.logger.info(f'Opened GPS sensor: "{name}"')

    def _close_GPS(self) -> None:
        data = dict(type='CloseGPS')
        data['name'] = self.name
        data['vid'] = self.vehicle.vid
        self.bng._send(data).ack('ClosedGPS')
        self.logger.info(f'Closed GPS sensor: "{self.name}"')

    def _poll_GPS_GE(self) -> StrDict:
        return self._send_sensor_request('PollGPSGE', ack='PolledGPSGECompleted', name=self.name)['data']

    def _poll_GPS_VE(self) -> StrDict:
        if not self.vehicle.connection:
            raise BNGError('The vehicle is not connected!')
        return send_sensor_request(self.vehicle.connection, 'PollGPSVE', name=self.name, sensorId=self.sensorId)['data']
