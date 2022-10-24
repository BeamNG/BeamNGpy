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

from logging import DEBUG, getLogger

from beamngpy.beamngcommon import LOGGER_ID

class Powertrain:
    def __init__(self, name, bng, vehicle, gfx_update_time=0.1, physics_update_time=0.015, is_send_immediately=False):
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
        bng.open_powertrain(name, vehicle, gfx_update_time, physics_update_time, is_send_immediately)

        # Fetch the unique Id number (in the simulator) for this powertrain sensor.  We will need this later.
        self.sensorId = int(bng.get_powertrain_id(name)['data'])
        self.logger.debug('Powertrain - sensor created: 'f'{self.name}')

    def remove(self):
        """
        Removes this sensor from the simulation.
        """

        # Remove this sensor from the simulation.
        self.bng.close_powertrain(self.name, self.vehicle)
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
            readings_data = self.bng.poll_powertrain_VE(self.name, self.vehicle, self.sensorId)['data']   # Get the most-recent single reading from vlua.
        else:
            readings_data = self.bng.poll_powertrain_GE(self.name)['data']                                # Get the bulk data from ge lua.

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
        return self.bng.send_ad_hoc_request_powertrain(self.name, self.vehicle.vid)['data']

    def is_ad_hoc_poll_request_ready(self, request_id):
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (bool): A flag which indicates if the ad-hoc polling request is complete.
        """

        self.logger.debug('Powertrain - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self.bng.is_ad_hoc_poll_request_ready_powertrain(request_id)

    def collect_ad_hoc_poll_request(self, request_id):
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (dict): The readings data.
        """

        readings = self.bng.collect_ad_hoc_poll_request_powertrain(request_id)['data']
        self.logger.debug('Powertrain - ad-hoc polling request returned and processed: 'f'{self.name}')
        return readings

    def set_requested_update_time(self, requested_update_time):
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time (float): The new requested update time.
        """

        self.bng.set_powertrain_requested_update_time(self.name, self.vehicle.vid, requested_update_time)
