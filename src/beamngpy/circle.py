import math

class circle:
    """
    A class for representing circles in R^2.
    """

    def __init__(self, x, y, r, dir):
        """
        Creates a circle.

        Args:
            x: The circle center x-coordinate.
            y: The circle center y-coordinate.
            r: The circle radius.
            dir: The circle direction of travel [1 = clockwise, -1 - anti-clockwise].
        """
        self.x = x                                                      # Center x-coordinate.
        self.y = y                                                      # Center y-coordinate.
        self.r = r                                                      # Radius.
        self.dir = dir                                                  # Direction of travel around circle { 1 = clockwise, -1 = anti-clockwise }.
        self.circumference = 2 * math.pi * self.r                       # Circumference.
        self.k = dir / max(r, 1e-24)                                    # Curvature.

    def _mag(self, x, y) -> float:
        """
        Computes the magnitude of a vector.

        Args:
            x: The vector x component.
            y: The vector y component.

        Returns:
            The magnitude (L^2 norm) of the given vector.
        """
        return math.sqrt(x * x + y * y)

    def _dot(self, x1, y1, x2, y2) -> float:
        """
        Computes the dot product of two vectors.

        Args:
            x1: The first vector x-coordinate.
            y1: The first vector y-coordinate.
            x2: The second vector x-coordinate.
            y2: The second vector y-coordinate.

        Returns:
            The dot product of the two given vectors.
        """
        return x1 * x2 + y1 * y2

    def _norm1(self, x1, y1, x2, y2) -> float:
        """
        Computes the L^1 distance between two points.

        Args:
            x1: The first point x-coordinate.
            y1: The first point y-coordinate.
            x2: The second point x-coordinate.
            y2: The second point y-coordinate.

        Returns:
            The L^1 distance between the two given points.
        """
        dx = x2 - x1
        dy = y2 - y1
        return dx * dx + dy * dy

    def _norm2(self, x1, y1, x2, y2) -> float:
        """
        Computes the L^2 distance (Euclidean distance) between two points.

        Args:
            x1: The first point x-coordinate.
            y1: The first point y-coordinate.
            x2: The second point x-coordinate.
            y2: The second point y-coordinate.

        Returns:
            The L^2 distance (Euclidean distance) between the two given points.
        """
        return self._mag(x2 - x1, y2 - y1)

    def _normalize(self, x, y) -> list[float]:
        """
        Normalizes a vector.

        Args:
            x: The vector x component.
            y: The vector y component.

        Returns:
            Normalizes the given vector.
        """
        mag_recip = 1.0 / self._mag(x, y)
        return [x * mag_recip, y * mag_recip]

    def arc_length(self, x1, y1, x2, y2) -> float:
        """
        Computes the shortest arc-length between two points on this circle.
        Points are expected to lie on the circle circumference.

        Args:
            x1: The first point x-coordinate.
            y1: The first point y-coordinate.
            x2: The second point x-coordinate.
            y2: The second point y-coordinate.

        Returns:
            Normalizes the given vector.
        """
        v1 = self._normalize(x1 - self.x, y1 - self.y)
        v2 = self._normalize(x2 - self.x, y2 - self.y)
        short_angle = math.acos(self._dot(v1[0], v1[1], v2[0], v2[1]))
        return (short_angle / (2.0 * math.pi)) * self.circumference

    def hdg(self, x1, y1, tx, ty) -> float:
        """
        Computes the heading at a point on this circle, with respect to another point.

        Args:
            x1: The point x-coordinate.
            y1: The point y-coordinate.
            tx: The x-coordinate of the context point (where the heading should point towards).
            ty: The y-coordinate of the context point (where the heading should point towards).

        Returns:
            An angle in radians, in range [-pi, pi], which represents the heading.
        """
        vx = self.y - y1                                                # Vector perpendicular to radial.
        vy = x1 - self.x
        mag = self._mag(vx, vy)
        nx = vx / mag                                                   # Normalised.
        ny = vy / mag
        ex1 = nx * 1e-5                                                 # Epsilon-lengthed vector.
        ey1 = ny * 1e-5
        d1 = self._norm1(x1 + ex1, y1 + ey1, tx, ty)                    # Distances between points along normal from p1, to p2.
        d2 = self._norm1(x1 - ex1, y1 - ey1, tx, ty)
        if d1 < d2:                                                     # Choose the normal which points closest to next point, p2.
            return math.atan2(vy, vx)                                   # Compute its heading (in radians).
        return math.atan2(-vy, -vx)

    def _compute_dir(self, x1, y1, tx, ty) -> float:
        """
        Computes the direction of a point on this circle, with respect to another point.

        Args:
            x1: The point x-coordinate.
            y1: The point y-coordinate.
            tx: The x-coordinate of the context point (where the heading should point towards).
            ty: The y-coordinate of the context point (where the heading should point towards).

        Returns:
            The direction of travel [1 if clockwise, -1 if anti-clockwise] to become closer to the given context point.
        """
        vx = self.y - y1                                                # Vector perpendicular to radial.
        vy = x1 - self.x
        mag = self._mag(vx, vy)
        nx = vx / mag                                                   # Normalised.
        ny = vy / mag
        ex1 = nx * 1e-5                                                 # Epsilon-lengthed vector.
        ey1 = ny * 1e-5
        d1 = self._norm1(x1 + ex1, y1 + ey1, tx, ty)                    # Distances between points along normal from p1, to p2.
        d2 = self._norm1(x1 - ex1, y1 - ey1, tx, ty)
        if d1 < d2:                                                     # Choose the normal which points closest to next point, p2.
            return 1.0                                                  # if the next point is further in direction 1, then direction should be flipped.
        return -1.0

    @staticmethod
    def circle_from_3_points(x1, y1, x2, y2, x3, y3):
        """
        Computes the unique circle which passes through three points, if it exists.

        Args:
            x1: The first point x-coordinate.
            y1: The first point y-coordinate.
            x2: The second point x-coordinate.
            y2: The second point y-coordinate.
            x3: The third point x-coordinate.
            y3: The third point y-coordinate.

        Returns:
            A tuple containing the unique circle passing through the three points, and a flag indicating if it exists.
            The circle will not exist if the three points are collinear, in which case we return False with a dummy circle.
        """
        z1 = complex(x1, y1)
        z2 = complex(x2, y2)
        z3 = complex(x3, y3)
        w = (z3 - z1) / (z2 - z1)
        if abs(w.imag) <= 1e-12:                                        # If points are collinear, leave early (avoids complex division by zero).
            return (circle(0, 0, 0, 1), True)
        c = (z2 - z1) * (w - abs(w)**2) / (2j * w.imag) + z1
        radius = abs(z1 - c)
        temp = circle(c.real, c.imag, radius, 1.0)                      # A temporary circle which may have the wrong direction.
        dir = temp._compute_dir(x1, y1, x2, y2)                         # Now determine the direction and return a circle which uses that.
        return (circle(c.real, c.imag, radius, dir), False)