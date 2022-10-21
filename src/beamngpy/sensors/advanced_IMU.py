"""
An interactive, automated IMU sensor, which produces regular acceleration and gyroscopic measurements in a local coordinate space.
This sensor can be attached to a vehicle, or can be fixed to a position in space. The dir and up parameters are used to set the local coordinate system.
A requested update rate can be provided, to tell the simulator how often to read measurements for this sensor. If a negative value is provided, the sensor
will not update automatically at all. However, ad-hoc polling requests can be sent at any time, even for non-updating sensors.
"""

from logging import DEBUG, getLogger

from beamngpy.beamngcommon import LOGGER_ID


class Advanced_IMU:
    def __init__(
            self, name, bng, vehicle, gfx_update_time=0.1, physics_update_time=0.015, pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1), window_width=None,
            frequency_cutoff=None, is_send_immediately=False, is_using_gravity=False, is_visualised=True, is_snapping_desired=False,
            is_force_inside_triangle=False):
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
        bng.open_advanced_IMU(name, vehicle, gfx_update_time, physics_update_time, pos, dir, up, window_width, is_send_immediately,
            frequency_cutoff, is_using_gravity, is_visualised, is_snapping_desired, is_force_inside_triangle)
        ddd = bng.get_advanced_imu_id(name)
        print("ddd: ", ddd)
        ddd = bng.get_advanced_imu_id(name)
        print("ddd: ", ddd)
        ddd = bng.get_advanced_imu_id(name)
        print("ddd: ", ddd)
        ddd = bng.get_advanced_imu_id(name)
        print("ddd: ", ddd)
        ddd = bng.get_advanced_imu_id(name)
        print("ddd: ", ddd)
        self.sensorId = bng.get_advanced_imu_id(name)['data']
        print("ID: ",self.sensorId)
        self.logger.debug('Advanced IMU - sensor created: 'f'{self.name}')

    def remove(self):
        """
        Removes this sensor from the simulation.
        """

        # Remove this sensor from the simulation.
        self.bng.close_advanced_IMU(self.name, self.vehicle)
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
            readings_data = self.bng.poll_advanced_IMU_VE(self.name)['data']    # We get the most-recent single reading data from vlua.
        else:
            readings_data = self.bng.poll_advanced_IMU_GE(self.name)['data']    # We get the bulk data from ge lua.

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
        return self.bng.send_ad_hoc_request_advanced_IMU(self.name, self.vehicle.vid)['data']

    def is_ad_hoc_poll_request_ready(self, request_id):
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (bool): A flag which indicates if the ad-hoc polling request is complete.
        """

        self.logger.debug('Advanced IMU - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self.bng.is_ad_hoc_poll_request_ready_advanced_IMU(request_id)

    def collect_ad_hoc_poll_request(self, request_id):
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id (int): The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            (dict): The readings data.
        """

        readings = self.bng.collect_ad_hoc_poll_request_advanced_IMU(request_id)['data']
        self.logger.debug('Advanced IMU - ad-hoc polling request returned and processed: 'f'{self.name}')

        return readings

    def get_position(self):
        """
        Gets the current world-space position of this sensor.

        Returns:
            (list): The sensor position.
        """
        table = self.bng.get_advanced_IMU_sensor_position(self.name)['data']
        return [table['x'], table['y'], table['z']]

    def get_direction(self):
        """
        Gets the current direction vector of this sensor.

        Returns:
            (list): The sensor direction.
        """
        table = self.bng.get_advanced_IMU_sensor_direction(self.name)['data']
        return [table['x'], table['y'], table['z']]

    def set_requested_update_time(self, requested_update_time):
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time (float): The new requested update time.
        """

        self.bng.set_advanced_IMU_requested_update_time(self.name, self.vehicle.vid, requested_update_time)

    def set_is_using_gravity(self, is_using_gravity):
        """
        Sets whether this sensor is to include gravity in the computation or not.

        Args:
            is_visualised(bool): A flag which indicates if this sensor is to use gravity in the computation or not.
        """

        self.bng.set_advanced_IMU_is_using_gravity(self.name, self.vehicle.vid, is_using_gravity)

    def set_is_visualised(self, is_visualised):
        """
        Sets whether this sensor is to be visualised or not.

        Args:
            is_visualised(bool): A flag which indicates if this sensor is to be visualised or not.
        """

        self.bng.set_advanced_IMU_is_visualised(self.name, self.vehicle.vid, is_visualised)
