import math

class vec3:
    """
    A class for storing vectors in R^3.
    """

    def __init__(self, x, y, z):
        """
        Creates a vec2 instance.

        Args:
            x: The x component.
            y: The y component.
            z: The z component.
        """
        self.x = x
        self.y = y
        self.z = z

    def add(self, b):
        """
        Adds a given vector to this vector.

        Args:
            b: the vector to add to this vector.

        Return:
            The addition of the two vectors.
        """
        return vec3(self.x + b.x, self.y + b.y, self.z + b.z)

    def sub(self, b):
        """
        Subtracts a given vector from this vector.

        Args:
            b: the vector to be subtracted from this vector.

        Return:
            The subtraction of the two vectors.
        """
        return vec3(self.x - b.x, self.y - b.y, self.z - b.y)

    def scmult(self, b):
        """
        Scalar multiplies a given scalar value to this vector.

        Args:
            b: the scalar value to be multiplied to this vector.

        Return:
            The vector * scalar product.
        """
        return vec3(self.x * b, self.y * b, self.z * b)

    def scdiv(self, b):
        """
        Scalar divides a scalar value to this vector.

        Args:
            b: the scalar value by which this vector is to be divided.

        Return:
            The vector / scalar product.
        """
        inv = 1.0 / b
        return vec3(self.x * inv, self.y * inv, self.z * inv)

    def dot(self, b) -> float:
        """
        Computes the dot product between this vector and a given vector.

        Args:
            b: the given second vector with which to compute the dot product.

        Returns:
            The scalar-valued dot product.
        """
        return self.x * b.x + self.y * b.y + self.z * b.z

    def cross(self, b):
        """
        Computes the cross product between this vector and a given vector

        Args:
            b: the given second vector with which to compute the cross product.

        Return:
            The cross product vector.
        """
        return vec3(self.y * b.z - self.z * b.y, -(self.x * b.z - self.z * b.x), self.x * b.y - self.y * b.x)

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
        dz = b.z - self.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def mag(self) -> float:
        """
        Computes the magnitude of this vector |v|.

        Return:
            The magnitude (scalar) of this vector.
        """
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self):
        """
        Normalizes this vector so that it becomes unit length (magnitude = 1).

        Return:
            The normalized, unit vector.
        """
        mag_inv = 1.0 / self.mag()
        return vec3(self.x * mag_inv, self.y * mag_inv, self.z * mag_inv)