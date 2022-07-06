"""
An interactive, automated accelerometer sensor, which produces regular acceleration measurements in a local coordinate space.
This sensor can be attached to a vehicle, or can be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.
"""

from logging import DEBUG as DBG_LOG_LEVEL, getLogger
from .beamngcommon import LOGGER_ID

class Accelerometer:

    def __init__(self, name, bng, vehicle, requested_update_time=0.1, pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1), is_using_gravity=False, 
        is_visualised=True, is_snapping_desired=False, is_force_inside_triangle=False):

        """
        Creates an accelerometer sensor.

        Args:
            name (str): A unique name for this accelerometer sensor.
            bng (BeamNGpy): The BeamNGpy instance, with which to communicate to the simulation.
            vehicle (Vehicle class): The vehicle to which this sensor should be attached. Note: a vehicle must be provided for the accelerometer.
            requested_update_time (float): The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
            pos (tuple): (X, Y, Z) Coordinate triplet specifying the position of the sensor, in world space.
            dir (tuple): (X, Y, Z) Coordinate triplet specifying the forward direction of the sensor.
            up (tuple): (X, Y, Z) Coordinate triplet specifying the up direction of the sensor.
            is_using_gravity (bool): A flag which indicates whether this sensor should consider acceleration due to gravity in its computations, or not.
            is_visualised (bool): Whether or not to render the ultrasonic sensor points in the simulator.
            is_snapping_desired (bool): A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle (not used for static sensors).
            is_force_inside_triangle (bool): A flag which indicates if the sensor should be forced inside the nearest vehicle triangle (not used for static sensors).
        """

        self.logger = getLogger(f'{LOGGER_ID}.Accelerometer')
        self.logger.setLevel(DBG_LOG_LEVEL)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name
        self.vid = vehicle.vid

        # Create and initialise this sensor in the simulation.
        bng.open_accelerometer(name, self.vid, requested_update_time, pos, dir, up, is_using_gravity, is_visualised, is_snapping_desired, is_force_inside_triangle)
        self.logger.debug('Accelerometer - sensor created: 'f'{self.name}')

    def remove(self):
        """
        Removes this sensor from the simulation.
        """

        # Remove this sensor from the simulation.
        self.bng.close_accelerometer(self.name, self.vid)
        self.logger.debug('Accelerometer - sensor removed: 'f'{self.name}')

    def poll(self):
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            (dict): A dictionary containing the acceleration in each dimension of the local coordinate system of the accelerometer sensor.
        """

        # Send and receive a request for readings data from this sensor.
        acceleration_data = self.bng.poll_accelerometer(self.name)['data']
        self.logger.debug('Accelerometer - sensor readings received from simulation: 'f'{self.name}')

        return acceleration_data

    def send_ad_hoc_poll_request(self):
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            (int): A unique Id number for the ad-hoc request.
        """

        self.logger.debug('Accelerometer  - ad-hoc polling request sent: 'f'{self.name}')
        return self.bng.send_ad_hoc_request_accelerometer(self.name, self.vid)['data']

    def is_ad_hoc_poll_request_ready(self, request_id):
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (bool): A flag which indicates if the ad-hoc polling request is complete.
        """

        self.logger.debug('Accelerometer  - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self.bng.is_ad_hoc_poll_request_ready_accelerometer(request_id)

    def collect_ad_hoc_poll_request(self, request_id):  
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (dict): The readings data.
        """

        readings = self.bng.collect_ad_hoc_poll_request_accelerometer(request_id)['data'] 
        self.logger.debug('Accelerometer  - ad-hoc polling request returned and processed: 'f'{self.name}')

        return readings

    def get_position(self):
        """
        Gets the current world-space position of this sensor.

        Returns:
            (list): The sensor position.
        """
        table = self.bng.get_accelerometer_sensor_position(self.name)['data']
        return [ table['x'], table['y'], table['z'] ]

    def get_direction(self):
        """
        Gets the current direction vector of this sensor.

        Returns:
            (list): The sensor direction.
        """
        table = self.bng.get_accelerometer_sensor_direction(self.name)['data']
        return [ table['x'], table['y'], table['z'] ]

    def set_requested_update_time(self, requested_update_time):
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time (float): The new requested update time.
        """

        self.bng.set_accelerometer_requested_update_time(self.name, self.vid, requested_update_time)

    def set_is_using_gravity(self, is_using_gravity):
        """
        Sets whether this accelerometer sensor is to include gravity in the computation or not.

        Args:
            is_visualised(bool): A flag which indicates if this accelerometer sensor is to use gravity in the computation or not.
        """

        self.bng.set_accelerometer_is_using_gravity(self.name, self.vid, is_using_gravity)

    def set_is_visualised(self, is_visualised):
        """
        Sets whether this accelerometer sensor is to be visualised or not.

        Args:
            is_visualised(bool): A flag which indicates if this accelerometer sensor is to be visualised or not.
        """

        self.bng.set_accelerometer_is_visualised(self.name, self.vid, is_visualised)