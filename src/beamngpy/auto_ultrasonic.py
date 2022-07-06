"""
.. module:: ultrasonic
    :platform: Windows
    :synopsis: Module containing the ultrasonic sensor.
    :noindex:

.. moduleauthor:: Dave Stark <dstark@beamng.gmbh>

An interactive, managed, automated ultrasonic sensor, which produces regular distance measurements, ready for further processing.
This sensor can be attached to a vehicle, or can be fixed to a position in space.
"""

from logging import DEBUG as DBG_LOG_LEVEL
from logging import getLogger

from .beamngcommon import LOGGER_ID

class Auto_Ultrasonic:

    def __init__(self, name, bng, vehicle=None, requested_update_time=0.1, update_priority=0.0, pos=(0, 0, 1.7), dir=(0, -1, 0), size=(200, 200),
        field_of_view=(0.1, 0.1), near_far_planes=(0.1, 5.1), range_roundess=-1.15, range_cutoff_sensitivity=0.0, range_shape=0.3, range_focus=0.376, 
        range_min_cutoff=0.1, range_direct_max_cutoff=5.0, sensitivity=3.0, fixed_window_size=10, is_visualised=True, is_static=False, 
        is_snapping_desired=False, is_force_inside_triangle=False):

        """
        Creates an ultrasonic sensor.

        Args:
            name (str): A unique name for this ultrasonic sensor.
            bng (BeamNGpy): The BeamNGpy instance, with which to communicate to the simulation.
            vehicle (Vehicle class): The vehicle to which this sensor should be attached, if any.
            requested_update_time (float): The time which should pass between sensor reading updates, in seconds. This is just a suggestion to the manager.
            update_priority (float): The priority which the sensor should ask for new readings. lowest -> 0, highest -> 1. 
            pos (tuple): (X, Y, Z) Coordinate triplet specifying the position of the sensor, in world space.
            dir (tuple): (X, Y, Z) Coordinate triplet specifying the direction of the sensor.
            size (tuple): (X, Y) The resolution of the sensor (the size of the depth buffer image in the distance measurement computation).
            field_of_view (tuple): (X, Y) The sensor field of view parameters.
            near_far_planes (tuple): (X, Y) The sensor near and far plane distances.
            range_roundness (float): the general roudness of the ultrasonic sensor range-shape. Can be negative.
            range_cutoff_sensitivity (float): a cutoff sensitivity parameter for the ultrasonic sensor range-shape.
            range_shape (float): the shape of the ultrasonic sensor range-shape in [0, 1], from conical to circular.
            range_focus (float): the focus parameter for the ultrasonic sensor range-shape.
            range_min_cutoff (float): the minimum cut-off distance for the ultrasonic sensor range-shape. Nothing closer than this will be detected.
            range_direct_max_cutoff (float): the maximum cut-off distance for the ultrasonic sensor range-shape. 
                This parameter is a hard cutoff - nothing further than this will be detected, although other parameters can also control the max distance.
            sensitivity (float): an ultrasonic sensor sensitivity parameter.
            fixed_window_size (float): an ultrasonic sensor sensitivity parameter.
            isVisualised (bool): Whether or not to render the ultrasonic sensor points in the simulator.
            is_static (bool): A flag which indicates whether this sensor should be static (fixed position), or attached to a vehicle.
            is_snapping_desired (bool): A flag which indicates whether or not to snap the sensor to the nearest vehicle triangle (not used for static sensors).
            is_force_inside_triangle (bool): A flag which indicates if the sensor should be forced inside the nearest vehicle triangle (not used for static sensors).
        """

        self.logger = getLogger(f'{LOGGER_ID}.Ultrasonic')
        self.logger.setLevel(DBG_LOG_LEVEL)

        # Cache some properties we will need later.
        self.bng = bng
        self.name = name

        # Create and initialise this sensor in the simulation.
        bng.open_ultrasonic(name, vehicle, requested_update_time, update_priority, pos, dir, size, field_of_view, near_far_planes, range_roundess,
            range_cutoff_sensitivity, range_shape, range_focus, range_min_cutoff, range_direct_max_cutoff, sensitivity, fixed_window_size,
            is_visualised, is_static, is_snapping_desired, is_force_inside_triangle)
        self.logger.debug('Ultrasonic - sensor created: 'f'{self.name}')

    def remove(self):
        """
        Removes this sensor from the simulation.
        """

        # Remove this sensor from the simulation.
        self.bng.close_ultrasonic(self.name)
        self.logger.debug('Ultrasonic - sensor removed: 'f'{self.name}')

    def poll(self):
        """
        Gets the most-recent readings for this sensor.

        Returns:
            (dict): A dictionary containing the distance measurement and the window (min and mix values) in which it was computed.
        """

        # Send and receive a request for readings data from this sensor.
        distance_measurement = self.bng.poll_ultrasonic(self.name, self.is_using_shared_memory)['data']
        self.logger.debug('Ultrasonic - sensor readings received from simulation: 'f'{self.name}')

        return distance_measurement

    def get_requested_update_time(self):
        """
        Gets the current 'requested update time' value for this sensor.
        """
        pass

    def get_update_priority(self):
        """
        Gets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, highest to lowest.
        """
        pass

    def get_position(self):
        """
        Gets the current world-space position of this sensor.
        """
        pass

    def get_direction(self):
        """
        Gets the current direction vector of this sensor.
        """
        pass

    def get_max_pending_requests(self):
        """
        Gets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.
        """
        pass

    def set_requested_update_time(self):
        """
        Sets the current 'requested update time' value for this sensor.
        """
        pass

    def set_update_priority(self):
        """
        Sets the current 'update priority' value for this sensor, in range [0, 1], with priority going 0 --> 1, , highest to lowest.
        """
        pass

    def set_max_pending_requests(self):
        """
        Sets the current 'max pending requests' value for this sensor. This is the maximum number of polling requests which can be issued at one time.
        """
        pass
