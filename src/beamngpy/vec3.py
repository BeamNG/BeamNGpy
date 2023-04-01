import math

class vec3:
    """
    A class for storing vectors in R^3. Contains functions for operating within that vector space.
    Can also be used as a vec2 class, since the z component is optional.
    """
    def __init__(self, x, y, z = 0.0):
        """
        Constructs an instance of vec3.

        Args:
            x: The vector x-coordinate.
            y: The vector y-coordinate.
            z (optional): The vector z-coordinate.
        """
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, b):
        """
        Vector addition.

        Args:
            b: A given vector to be added to this vector.

        Returns:
            The sum of the two vectors.
        """
        return vec3(self.x + b.x, self.y + b.y, self.z + b.z)

    def __sub__(self, b):
        """
        Vector subtraction.

        Args:
            b: A given vector to be subtracted from this vector.

        Returns:
            The difference of the two vectors.
        """
        return vec3(self.x - b.x, self.y - b.y, self.z - b.z)

    def __mul__(self, b):
        """
        Scalar multiplication.

        Args:
            b: A given scalar value to be multiplied to this vector (from either left or right).

        Returns:
            This vector multiplied by the given scalar value.
        """
        return vec3(self.x * b, self.y * b, self.z * b)

    __rmul__ = __mul__

    def __truediv__(self, b):
        """
        Scalar division.

        Args:
            b: A given scalar value by which to divide this vector.

        Returns:
            This vector divided by the given scalar value.
        """
        inv = 1.0 / b
        return vec3(self.x * inv, self.y * inv, self.z * inv)

    def dot(self, b) -> float:
        """
        The dot product between this vector and a given vector.

        Args:
            b: The given vector.

        Returns:
            The dot product between the two vectors (a scalar value).
        """
        return self.x * b.x + self.y * b.y + self.z * b.z

    def cross(self, b):
        """
        The cross product between this vector and a given vector.

        Args:
            b: The given vector.

        Returns:
            The cross product between the two vectors (a vector value)
        """
        return vec3(self.y * b.z - self.z * b.y, -(self.x * b.z - self.z * b.x), self.x * b.y - self.y * b.x)

    def distance(self, b) -> float:
        """
        The L^2 (Euclidean) distance between this vector and a given vector. AKA the distance formula.

        Args:
            b: The given vector.

        Returns:
            This L^2 (Euclidean) distance between the two vectors (a scalar value).
        """
        dx = b.x - self.x
        dy = b.y - self.y
        dz = b.z - self.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def length(self) -> float:
        """
        The length (magnitude) of this vector. [ ie length := |vector| ]

        Returns:
            The length of this vector (a scalar value).
        """
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self):
        """
        Normalizes this vector so that it becomes unit length (length = 1).

        Returns:
            The normalized vector.
        """
        mag_inv = 1.0 / self.length()
        return vec3(self.x * mag_inv, self.y * mag_inv, self.z * mag_inv)
