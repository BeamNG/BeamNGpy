import math
from collections import deque

class Trajectory:

    def __init__(self, memory = 2000, x_min = -1000.0, x_max = 1000.0, y_min = -1000.0, y_max = 1000.0, origin_x = 900.0, origin_y = 500.0, zoom = 1.0, rot_deg = 0.0):
        """
        Constructs a trajectory manager/viewer instance.
        This is used to store the trajectory of a given vehicle over time, and to plot it using a graphics API.

        Args:
            memory (int): The size of the trajectory queue used by this instance. This is how many previous positions we store.
            x_min (float): The left-most display position coordinate.
            x_max (float): The right-most display position coordinate.
            y_min (float): The bottom-most display position coordinate.
            y_max (float): The top-most display position coordinate.
            origin_x (float): The x-coordinate of the center of the display (this is where the most recent point of the trajectory will appear on the display).
            origin_y (float): The y-coordinate of the center of the display (this is where the most recent point of the trajectory will appear on the display).
            zoom (float): The multiplicative factor by which to zoom into the trajectory at the origin position.
            rot_deg (float): The angle by which to rotate the trajectory by, in degrees.
        """

        # Trajectory queue properties.
        self.trajectory = deque()                                                                       # The trajectory queue of the associated vehicle.
        self.memory = memory                                                                            # The size (memory) of the trajectory queue.
        self.x_min, self.x_max, self.y_min, self.y_max = x_min, x_max, y_min, y_max                     # The display ranges.
        self.origin_x, self.origin_y = origin_x, origin_y                                               # The centre of the display region.
        self.zoom = zoom                                                                                # The factor by which to scale the trajectory by.
        rot_rad = rot_deg * 0.0174533                                                                   # The angle by which to rotate the trajectory by, in radians.
        self.sine, self.cosine = math.sin(rot_rad), math.cos(rot_rad)                                # The sine/cosine of the rotation angle.

        # Miscellaneous.
        self.is_pause = False

    def _is_in_range(self, x, y):
        """
        Determines if a given point is within the prescribed range of this trajectory manager/display instance.

        Args:
            x (float): The x-coordinate of the point.
            y (float): The y-coordinate of the point.

        Return:
            bool: True if the point is within the range, otherwise False.
        """
        return x >= self.x_min and x <= self.x_max and y >= self.y_min and y <= self.y_max

    def _rotate(self, x, y):
        """
        Rotates a point (x, y) around the origin (0, 0) by the prescribed angle.

        Args:
            x (float): The x-coordinate of the point to be rotated.
            y (float): The y-coordinate of the point to be rotated.

        Return:
            list: The rotated point [x, y].
        """
        return [x * self.cosine - y * self.sine, x * self.sine + y * self.cosine]

    def pause(self, is_pause):
        """
        Sets whether to pause this time series instance or not. If paused, no updates will be performed and the data will remain frozen on screen.

        Args:
            is_pause (bool): Pauses the time series instance if True, otherwise sets it to continue updating.
        """
        self.is_pause = is_pause

    def reset(self):
        """
        Resets the trajectory queue to start recording from the current position (all memory is removed from trajectory queue).
        """
        self.trajectory = deque()

    def update(self, pos):
        """
        Update this trajectory manager/viewer with the latest positional information.

        Args:
            pos (list): The latest vehicle position, as a list of form [x, y, z].
        """
        # If we are pausing, do not perform any data updating.
        if self.is_pause == True:
            return

        # Update the trajectory queue with the latest vehicle position.
        pos_copy = [pos[0], pos[1], pos[2]]
        self.trajectory.append(pos_copy)
        if len(self.trajectory) > self.memory:
            self.trajectory.popleft()

    def display(self):
        """
        Gets all the display data for this trajectory manager/viewer.

        Return:
            list: A polyline representation of the trajectory, scaled and translated to the display parameters set in this trajectory manager/viewer.
        """
        lines = []
        if len(self.trajectory) > 0:
            last_pos = self.trajectory[-1]
            x_off, y_off = last_pos[0] - self.origin_x, last_pos[1] - self.origin_y                     # offset to put trajectory start at center of image.
            traj_lines = len(self.trajectory) - 1
            for i in range(traj_lines):
                p0 = self.trajectory[i]
                a11, b11 = self._rotate(p0[0] - x_off - self.origin_x, p0[1] - y_off - self.origin_y)
                x1 = a11 * self.zoom + self.origin_x
                y1 = b11 * self.zoom + self.origin_y

                p1 = self.trajectory[i + 1]
                c11, d11 = self._rotate(p1[0] - x_off - self.origin_x, p1[1] - y_off - self.origin_y)
                x2 = c11 * self.zoom + self.origin_x
                y2 = d11 * self.zoom + self.origin_y
                if self._is_in_range(x1, y1) and self._is_in_range(x2, y2):
                    lines.append([x1, y1, x2, y2])
        return lines