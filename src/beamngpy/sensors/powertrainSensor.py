"""
An interactive, automated powertrain sensor, which produces regular readings directly from a vehicle's powertrain.
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
    from ..beamng import BeamNGpy, Vehicle

class PowertrainSensor:
    def __init__(self, name: str, bng: BeamNGpy, vehicle: Vehicle, gfx_update_time=0.1, physics_update_time=0.015,
                 is_send_immediately=False):
        """
        Creates a powertrain sensor.

        Args:
            name (str): A unique name for this powertrain sensor.
            bng (BeamNGpy): The BeamNGpy instance, with which to communicate to the simulation.
            vehicle (Vehicle class): The vehicle to which this sensor should be attached. Note: a vehicle must be provided for the powertrain sensor.
            gfx_update_time (float): The gfx-step time which should pass between sensor reading updates to the user, in seconds.
            physics_update_time (float): The physics-step time which should pass between actual sampling the sensor, in seconds.
            is_send_immediately (bool): A flag which indicates if the readings should be sent back as soon as available or upon graphics step updates, as bulk.
        """

        self.logger = getLogger(f'{LOGGER_ID}.Powertrain')
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name
        self.vehicle = vehicle
        self.is_send_immediately = is_send_immediately

        # Create and initialise this sensor in the simulation.
        self._open_powertrain(name, vehicle, gfx_update_time, physics_update_time, is_send_immediately)

        # Fetch the unique Id number (in the simulator) for this powertrain sensor.  We will need this later.
        self.sensorId = int(self._get_powertrain_id())
        self.logger.debug('Powertrain - sensor created: 'f'{self.name}')

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
        self._close_powertrain()
        self.logger.debug('Powertrain - sensor removed: 'f'{self.name}')

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
            readings_data = self._poll_powertrain_VE()
        else:
            # Get the bulk data from ge lua.
            readings_data = self._poll_powertrain_GE()

        self.logger.debug('Powertrain - sensor readings received from simulation: 'f'{self.name}')
        return readings_data

    def send_ad_hoc_poll_request(self):
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            (int): A unique Id number for the ad-hoc request.
        """
        self.logger.debug('Powertrain - ad-hoc polling request sent: 'f'{self.name}')
        return self._send_sensor_request(
            'SendAdHocRequestPowertrain', ack='CompletedSendAdHocRequestPowertrain', name=self.name,
            vid=self.vehicle.vid)['data']

    def is_ad_hoc_poll_request_ready(self, request_id):
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (bool): A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug('Powertrain - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self._send_sensor_request('IsAdHocPollRequestReadyPowertrain',
                                         ack='CompletedIsAdHocPollRequestReadyPowertrain', requestId=request_id)['data']

    def collect_ad_hoc_poll_request(self, request_id):
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (dict): The readings data.
        """
        readings = self._send_sensor_request('CollectAdHocPollRequestPowertrain',
                                             ack='CompletedCollectAdHocPollRequestPowertrain', requestId=request_id)['data']
        self.logger.debug('Powertrain - ad-hoc polling request returned and processed: 'f'{self.name}')
        return readings

    @ack('CompletedSetPowertrainRequestedUpdateTime')
    def set_requested_update_time(self, requested_update_time):
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time (float): The new requested update time.
        """
        return self._set_sensor(
            'SetPowertrainRequestedUpdateTime', name=self.name, vid=self.vehicle.vid,
            GFXUpdateTime=requested_update_time)

    def _get_powertrain_id(self):
        return self._send_sensor_request('GetPowertrainId', ack='CompletedGetPowertrainId', name=self.name)['data']

    @ack('OpenedPowertrain')
    def _open_powertrain(self, name, vehicle, gfx_update_time, physics_update_time, is_send_immediately):
        data = dict(type='OpenPowertrain')
        data['name'] = name
        data['vid'] = vehicle.vid
        data['GFXUpdateTime'] = gfx_update_time
        data['physicsUpdateTime'] = physics_update_time
        data['isSendImmediately'] = is_send_immediately
        resp = self.bng.send(data)
        self.logger.info(f'Opened Powertrain sensor: "{name}"')
        return resp

    @ack('ClosedPowertrain')
    def _close_powertrain(self):
        data = dict(type='ClosePowertrain')
        data['name'] = self.name
        data['vid'] = self.vehicle.vid
        resp = self.bng.send(data)
        self.logger.info(f'Closed Powertrain sensor: "{self.name}"')
        return resp

    def _poll_powertrain_GE(self):
        return self._send_sensor_request('PollPowertrainGE', ack='PolledPowertrainGECompleted', name=self.name)['data']

    def _poll_powertrain_VE(self):
        if not self.vehicle.connection:
            raise BNGError('The vehicle is not connected!')
        return send_sensor_request(self.vehicle.connection, 'PollPowertrainVE', name=self.name, sensorId=self.sensorId)['data']
