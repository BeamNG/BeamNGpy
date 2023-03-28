import math

class vec2:
    """
    A class for storing vectors in R^2.
    """

    def __init__(self, x, y):
        """
        Creates a vec2 instance.

        Args:
            x: The x component.
            y: The y component.
        """
        self.x = x
        self.y = y

    def add(self, b):
        """
        Adds a given vector to this vector.

        Args:
            b: the vector to add to this vector.

        Return:
            The addition of the two vectors.
        """
        return vec2(self.x + b.x, self.y + b.y)

    def sub(self, b):
        """
        Subtracts a given vector from this vector.

        Args:
            b: the vector to be subtracted from this vector.

        Return:
            The subtraction of the two vectors.
        """
        return vec2(self.x - b.x, self.y - b.y)

    def scmult(self, b):
        """
        Scalar multiplies a given scalar value to this vector.

        Args:
            b: the scalar value to be multiplied to this vector.

        Return:
            The vector * scalar product.
        """
        return vec2(self.x * b, self.y * b)

    def scdiv(self, b):
        """
        Scalar divides a scalar value to this vector.

        Args:
            b: the scalar value by which this vector is to be divided.

        Return:
            The vector / scalar product.
        """
        inv = 1.0 / b
        return vec2(self.x * inv, self.y * inv)

    def dot(self, b) -> float:
        """
        Computes the dot product between this vector and a given vector.

        Args:
            b: the given second vector with which to compute the dot product.

        Returns:
            The scalar-valued dot product.
        """
        return self.x * b.x + self.y * b.y

    def cross(self, v) -> float:
        """
        Computes the cross product between this vector and a given vector

        Args:
            b: the given second vector with which to compute the cross product.

        Return:
            The scalar valued cross product (cross products are scalar in R^2).
        """
        return self.x * v.y - self.y * v.x

    def L2(self, b) -> float:
        """
        Computes the L^2 (Euclidean) norm between this vector and a second given vector. AKA: the distance formula.

        Args:
            b: the given second vector.

        Return:
            The L^2 norm between the two vectors.
        """
        dx = b.x - self.x
        dy = b.y - self.y
        return math.sqrt(dx * dx + dy * dy)

    def mag(self) -> float:
        """
        Computes the magnitude of this vector |v|.

        Return:
            The magnitude (scalar) of this vector.
        """
        return math.sqrt(self.x * self.x + self.y * self.y)

    def normalize(self):
        """
        Normalizes this vector so that it becomes unit length (magnitude = 1).

        Return:
            The normalized, unit vector.
        """
        mag_inv = 1.0 / math.sqrt(self.x * self.x + self.y * self.y)
        return vec2(self.x * mag_inv, self.y * mag_inv)

    def hdg(self) -> float:
        """
        Computes the heading angle of this vector, in radians, in range [-pi, pi].

        Return:
            The heading angle of this vector.
        """
        return math.atan2(self.y, self.x)