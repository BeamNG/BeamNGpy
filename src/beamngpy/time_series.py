from collections import deque

class Time_Series:

    def __init__(self, size = 2000, x_min = 50.0, x_max = 350.0, y_min = 50.0, y_max = 150.0, grid_spacing_x = 10, grid_spacing_y = 5, data_min = -50.0, data_max = 50.0,
        axes_overlap_x = 10.0, axes_overlap_y = 10.0, grid_notch_x = 5.0, grid_notch_y = 5.0):
        """
        Constructs a time-series data manager and visualizer instance.
        On construction, the data size and various limits can be set, then when new data is generated, this can be updated using the 'update_data()' method.
        When we want to render the time-series, we call separate methods to get the axes lines, grid lines, and actual data polyline.
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

        self.size = size
        self.x_min, self.x_max, self.y_min, self.y_max = x_min, x_max, y_min, y_max
        self.x_range, self.y_range = self.x_max - self.x_min, self.y_max - self.y_min
        self.y_min, self.y_max = y_min, y_max
        self.data_min, self.data_max = data_min, data_max
        self.data_range_inv = 1.0 / (self.data_max - self.data_min)
        self.t = []                                                                                             # The x-axis coordinates of the data.
        self.data = deque()                                                                                     # The y-axis coordinates of the data.
        dt = self.x_range / max(1e-24, float(self.size))
        y_midpoint = (self.y_min + self.y_max) * 0.5
        for i in range(self.size):                                                                              # Default all data to center of plot.
            self.t.append(self.x_min + (i * dt))
            self.data.append(y_midpoint)

        # Compute the axes data.
        self.axes_overlap_x, self.axes_overlap_y = axes_overlap_x, axes_overlap_y
        self.axes_lines = self._compute_axes()

        # Compute the grid data.
        self.grid_spacing_x, self.grid_spacing_y = grid_spacing_x, grid_spacing_y
        self.grid_notch_x, self.grid_notch_y = grid_notch_x, grid_notch_y
        self.grid_lines = self._compute_grid()

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

    def update_data(self, data):
        """
        Updates this time series with a collection of new data. This data is appended to the end of the current data queue, and an equal amount of data is removed
        from the back of the queue such that the queue always remains the same size (ie has the same amount of memory).

        Args:
            data: The given collection of data values, in the form [d0, d1, d2, ...] where the d are scalar floats.
        """
        data_size = len(data)
        for i in range(data_size):
            self.data.popleft()                                                 # For every new entry we add, we remove one at the back, to always keep it the same size.
            self.data.append(self._scale_to_range(data[i]))

    def get_axes_lines(self):
        """
        Gets the horizontal and vertical axes lines for this time series.

        Returns:
            List: The horizontal [0] and vertical [1] axes lines, where each is in the format [x0, y0, x1, y1].
        """
        return self.axes_lines

    def get_grid_lines(self):
        """
        Gets the horizontal and vertical grid lines for this time series.

        Returns:
            Dict: Contains two collections of lines; one set of thicker-drawn lines, and another thinner-drawn set (the graphics API should handle this, if required).
        """
        return self.grid_lines

    def get_data_lines(self):
        """
        Gets the polyline data for this time series, ready to be rendered by a graphics API.

        Returns:
            List: The vertex buffer, where each vertex has the format [x0, y0].
        """
        v = []
        for i in range(self.size):
            v.append([self.t[i], self.data[i]])
        return v