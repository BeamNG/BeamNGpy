from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any

from beamngpy.logging import LOGGER_ID, BNGError
from beamngpy.types import StrDict

from .communication_utils import send_sensor_request, set_sensor

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

__all__ = ['roadsSensor']


class roadsSensor:
    """
    roadsSensor class
    A sensor which gives geometric and semantic data of the road; this data is the parametric cubic equations for the left and right roadedge 
    and the centerline, as well as 4 points of the centerline
    Args:
        name: A unique name for this roads sensor.
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this sensor should be attached. Note: a vehicle must be provided for the roads sensor.
        gfx_update_time: The gfx-step time which should pass between sensor reading updates to the user, in seconds.
        physics_update_time: The physics-step time which should pass between actual sampling the sensor, in seconds.
        is_send_immediately: A flag which indicates if the readings should be sent back as soon as available or upon graphics step updates, as bulk.
    """

    def __init__(self, name: str, bng: BeamNGpy, vehicle: Vehicle, gfx_update_time: float = 0.0, physics_update_time: float = 0.01,
                 is_send_immediately: bool = False):
        self.logger = getLogger(f'{LOGGER_ID}.roadsSensor')
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name
        self.vehicle = vehicle
        self.is_send_immediately = is_send_immediately

        # Create and initialise this sensor in the simulation.
        self._open_roadsSensor(name, vehicle, gfx_update_time, physics_update_time, is_send_immediately)

        # Fetch the unique Id number (in the simulator) for this roads sensor.  We will need this later.
        self.sensorId = self._get_roadsSensor_id()
        self.logger.debug('roadsSensor created: 'f'{self.name}')

    def _send_sensor_request(self, type: str, ack: str | None = None, **kwargs: Any) -> StrDict:
        if not self.bng.connection:
            raise BNGError('The simulator is not connected!')
        return send_sensor_request(self.bng.connection, type, ack, **kwargs)

    def _set_sensor(self, type: str, **kwargs: Any) -> None:
        if not self.bng.connection:
            raise BNGError('The simulator is not connected!')
        set_sensor(self.bng.connection, type, **kwargs)

    def remove(self) -> None:
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_roadsSensor()
        self.logger.debug('roadsSensor removed: 'f'{self.name}')

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
            readings_data = self._poll_roadsSensor_VE()
        else:
            # Get the bulk data from ge lua.
            readings_data = self._poll_roadsSensor_GE()

        self.logger.debug('roadsSensor readings received from simulation: 'f'{self.name}')
        return readings_data

    def send_ad_hoc_poll_request(self) -> int:
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            A unique Id number for the ad-hoc request.
        """
        self.logger.debug('roadsSensor - ad-hoc polling request sent: 'f'{self.name}')
        return int(self._send_sensor_request(
            'SendAdHocRequestRoadsSensor', ack='CompletedSendAdHocRequestRoadsSensor', name=self.name,
            vid=self.vehicle.vid)['data'])

    def is_ad_hoc_poll_request_ready(self, request_id: int) -> bool:
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug('roadsSensor - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self._send_sensor_request('IsAdHocPollRequestReadyRoadsSensor',
                                         ack='CompletedIsAdHocPollRequestReadyRoadsSensor', requestId=request_id)['data']

    def collect_ad_hoc_poll_request(self, request_id: int) -> StrDict:
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            The readings data.
        """
        readings = self._send_sensor_request('CollectAdHocPollRequestRoadsSensor',
                                             ack='CompletedCollectAdHocPollRequestRoadsSensor', requestId=request_id)['data']
        self.logger.debug('roadsSensor - ad-hoc polling request returned and processed: 'f'{self.name}')
        return readings

    def set_requested_update_time(self, requested_update_time: float) -> None:
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time: The new requested update time.
        """
        self._set_sensor(
            'SetRoadsSensorRequestedUpdateTime', ack='CompletedSetRoadsSensorRequestedUpdateTime', name=self.name, vid=self.vehicle.vid,
            GFXUpdateTime=requested_update_time)

    def _get_roadsSensor_id(self) -> int:
        return int(self._send_sensor_request('GetRoadsSensorId', ack='CompletedGetRoadsSensorId', name=self.name)['data'])

    def _open_roadsSensor(self, name: str, vehicle: Vehicle, gfx_update_time: float, physics_update_time: float, is_send_immediately: bool) -> None:
        data: StrDict = dict(type='OpenRoadsSensor')
        data['name'] = name
        data['vid'] = vehicle.vid
        data['GFXUpdateTime'] = gfx_update_time
        data['physicsUpdateTime'] = physics_update_time
        data['isSendImmediately'] = is_send_immediately
        self.bng._send(data).ack('OpenedRoadsSensor')
        self.logger.info(f'Opened RoadsSensor: "{name}"')

    def _close_roadsSensor(self) -> None:
        data = dict(type='CloseRoadsSensor')
        data['name'] = self.name
        data['vid'] = self.vehicle.vid
        self.bng._send(data).ack('ClosedRoadsSensor')
        self.logger.info(f'Closed roadsSensor: "{self.name}"')

    def _poll_roadsSensor_GE(self) -> StrDict:
        return self._send_sensor_request('PollRoadsSensorGE', ack='PolledRoadsSensorGECompleted', name=self.name)['data']

    def _poll_roadsSensor_VE(self) -> StrDict:
        if not self.vehicle.connection:
            raise BNGError('The vehicle is not connected!')
        return send_sensor_request(self.vehicle.connection, 'PollRoadsSensorVE', name=self.name, sensorId=self.sensorId)['data']
