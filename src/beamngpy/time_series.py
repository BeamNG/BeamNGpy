from collections import deque

class Time_Series:

    def __init__(self, size = 2000, x_min = 50.0, x_max = 350.0, y_min = 50.0, y_max = 150.0, grid_spacing_x = 10, grid_spacing_y = 5, data_min = -50.0, data_max = 50.0,
        axes_overlap_x = 10.0, axes_overlap_y = 10.0, grid_notch_x = 5.0, grid_notch_y = 5.0):
        """
        Constructs a time-series data manager and visualizer instance, for use with fixed-dt data.
        On construction, the data size and various limits can be set, then when new data is generated, this can be updated using the 'update()' method.
        When we want to render the time-series, we call the 'display()' method to fetch the latest geometric data (a vertex buffer and some lines, here).
        The time-series can be paused using the 'pause()' method, which will freeze data on the screen until pause is unset using the same method.
        It is up to the user of this class how this information is rendered and with which graphics API.

        Args:
            size (int): The number of data points to be displayed on a plot of this time-series.
            x_min (float): The x-coordinate of the vertical axis (ie the left of the plot rectangle).
            x_max (float): The right-most x-coordinate of the grid (ie the right of the plot rectangle).
            y_min (float): The y-coordinate of the horizontal axis (ie the bottom of the plot rectangle).
            y_max (float): The top-most y-coordinate of the grid (ie the top of the plot rectangle).
            grid_spacing_x (int): The number of x-dimension divisions on the grid.
            grid_spacing_y (int): The number of y-dimension divisions on the grid.
            data_min (float): The minimum data value which can be displayed on the plot (will be shown at the bottom limit of the plot).
            data_max (float): The maximum data value which can be displayed on the plot (will be shown at the top limit of the plot).
            axes_overlap_x (float): The amount of horizontal excess at the origin, when drawing the horizontal axis.
            axes_overlap_y (float): The amount of vertical excess at the origin, when drawing the vertical axis.
            grid_notch_x (float): The amount of excess on the horizontal grid lines (which protrude outside past the vertical axis).
            grid_notch_y (float): The amount of excess on the vertical grid lines (which protrude outside past the horizontal axis).
        """

        # Data parameters.
        self.size = size
        self.x_min, self.x_max, self.y_min, self.y_max = x_min, x_max, y_min, y_max
        self.x_range, self.y_range = self.x_max - self.x_min, self.y_max - self.y_min
        self.data_min, self.data_max = data_min, data_max
        self.data_range_inv = 1.0 / (self.data_max - self.data_min)
        self.t = []                                                                                             # The x-axis coordinates of the data.
        self.data = deque()                                                                                     # The y-axis coordinates of the data.
        self.times = deque()                                                                                    # The time-stamps for the data (optional use).
        dt = self.x_range / max(1e-24, float(self.size))
        y_midpoint = (self.y_min + self.y_max) * 0.5
        for i in range(self.size):                                                                              # Default all data to center of plot.
            self.t.append(self.x_min + (i * dt))
            self.data.append(y_midpoint)
            self.times.append(-1e12)

        # Compute the axes data.
        self.axes_overlap_x, self.axes_overlap_y = axes_overlap_x, axes_overlap_y
        self.axes_lines = self._compute_axes()

        # Compute the grid data.
        self.grid_spacing_x, self.grid_spacing_y = grid_spacing_x, grid_spacing_y
        self.grid_notch_x, self.grid_notch_y = grid_notch_x, grid_notch_y
        self.grid_lines = self._compute_grid()

        # Marking/pausing properties.
        self.is_marked = False
        self.mark_pos_in_queue = 1
        self.mark_index = 0
        self.is_marker_at_target = False
        self.is_pause = False

    def _compute_axes(self):
        """
        Compute the horizontal and vertical axes lines of the time-series plot.

        Returns:
            List: The horizontal [0] and vertical [1] axes lines, where each is in the format [x0, y0, x1, y1].
        """
        return [
            [self.x_min - self.axes_overlap_x, self.y_min, self.x_max + self.axes_overlap_x, self.y_min],       # Horizontal axis line.
            [self.x_min, self.y_min - self.axes_overlap_y, self.x_min, self.y_max + self.axes_overlap_y] ]      # Vertical axis line.

    def _compute_grid(self):
        """
        Compute the horizontal and vertical grid lines of the time-series plot.
        The lines alternate between being thick and thin, to make viewing/measuring on the rendered plot easier.

        Returns:
            Dict: Contains two collections of lines; one set of thicker-drawn lines, and another thinner-drawn set (the graphics API should handle this, if required).
        """
        # We alternative the grid between thick and thin lines, in both x and y.
        grid_thin, grid_thick = [], []
        is_line_thick = False

        # Compute the vertical grid lines (moving along x).
        dx = float(self.x_range) / float(self.grid_spacing_x)
        notched_min_y = self.y_min - self.grid_notch_y
        for i in range(1, self.grid_spacing_x + 1):
            gx = self.x_min + (i * dx)
            line = [gx, notched_min_y, gx, self.y_max]
            if is_line_thick == True:
                grid_thick.append(line)
            else:
                grid_thin.append(line)
            is_line_thick = not is_line_thick

        # Compute the horizontal grid lines (moving up y).
        dy = float(self.y_range) / float(self.grid_spacing_y)
        notched_min_x = self.x_min - self.grid_notch_x
        for i in range(1, self.grid_spacing_y + 1):
            gy = self.y_min + (i * dy)
            line = [notched_min_x, gy, self.x_max, gy]
            if is_line_thick == True:
                grid_thick.append(line)
            else:
                grid_thin.append(line)
            is_line_thick = not is_line_thick

        # Return data as a dictionary of thick and thin line lists.
        return { 'thick' : grid_thick, 'thin' : grid_thin }

    def _scale_to_range(self, val):
        """
        Scales and translates a given data value to the chosen range of this time-series. Data is snapped the given [data_min, data_max] range, then its
        coordinate on the y-axis is computed.

        Args:
            val: The given data value, in range [-inf, inf].

        Returns:
            float: The scaled and translated data value, now in range [y_min, y_max], ready to render at this coordinate.
        """
        val_snapped = max(self.data_min, min(self.data_max, val))
        ratio = (val_snapped - self.data_min) * self.data_range_inv
        return self.y_min + ratio * (self.y_range)

    def pause(self, is_pause):
        """
        Sets whether to pause this time series instance or not. If paused, no updates will be performed and the data will remain frozen on screen.

        Args:
            is_pause (bool): Pauses the time series instance if True, otherwise sets it to continue updating.
        """
        self.is_pause = is_pause

    def mark(self, idx = 300, sync_time = None):
        """
        Marks the time-series at the current instant in time, such that this marked point (the most-recently added item of data) runs until the idx position in the queue,
        then the time-series will pause.

        Args:
            idx (int): The index in the queue, at which to allow the marked position to run to, before pausing.
            sync_time (float): The time at which to sync the mark to (eg from another sensor). This is used to line up readings at the correct time.
        """
        size = len(self.data)
        self.is_marked = True
        self.mark_index = idx
        if sync_time is None:
            self.mark_pos_in_queue = size - 1
        else:
            closest_time = 1e12
            closest_idx = -1
            for i in range(size):
                dt = abs(sync_time - self.times[i])
                if dt < closest_time:
                    closest_time = min(closest_time, abs(sync_time - self.times[i]))
                    closest_idx = i
            if closest_idx >= 0:
                self.mark_pos_in_queue = closest_idx
            else:
                self.mark_pos_in_queue = size - 1

    def find_first_spike(self, tol = 10.0):
        """
        Finds the first data point, along t, which has a rate of change larger than the given tolerance.

        Args:
            tol (float): The given tolerance.

        Return:
            float: The x coordinate in the display space, at which the first spike in the data occurs. If none are found, the last point is returned.
        """
        for i in range(0, len(self.data) - 1):
            a, b = self.data[i], self.data[i + 1]
            dx = abs(b - a)
            if dx > tol:
                return self.t[i]
        return self.t[-1]

    def update(self, data, times = []):
        """
        Updates this time series with a collection of new data. This data is appended to the end of the current data queue, and an equal amount of data is removed
        from the back of the queue such that the queue always remains the same size (ie has the same amount of memory).
        NOTE: If the mesh visualizer is paused or marking is complete, the data will remain as it was previously.

        Args:
            data: The given collection of data values, in the form [d0, d1, d2, ...] where the d are scalar floats.
            times: The time-stamps for the data (optional). This is needed if syncing is required.
        """
        # If we are pausing or marking (with the marker having reached its target destination), do not perform any data updating.
        if self.is_pause == True or self.is_marker_at_target == True:
            return

        # Add each new entry one by one. As we add an entry, remove one at the back, so as to always keep the queue the same size.
        is_times = len(times) > 0
        data_size = len(data)
        for i in range(data_size):
            self.data.popleft()
            self.data.append(self._scale_to_range(data[i]))
            if is_times:
                self.times.popleft()
                self.times.append(times[i])
            if self.is_marked == True:                                      # Transport the marker until it reaches its destination (the stored mark index).
                self.mark_pos_in_queue = self.mark_pos_in_queue - 1
                if self.mark_pos_in_queue <= self.mark_index:
                    self.is_marker_at_target = True
                    return

    def display(self):
        """
        Gets all the display data for this time series.

        Returns:
            Dict: The axes lines, grid lines, and data polyline in individual keys.
        """
        disp = {}
        disp['axes'] = self.axes_lines
        disp['grid'] = self.grid_lines
        v = []
        for i in range(self.size):
            v.append([self.t[i], self.data[i]])
        disp['data'] = v
        return disp