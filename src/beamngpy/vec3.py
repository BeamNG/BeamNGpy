import math

class vec3:
    def __init__(self, x, y, z = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, b):
        return vec3(self.x + b.x, self.y + b.y, self.z + b.z)

    def __sub__(self, b):
        return vec3(self.x - b.x, self.y - b.y, self.z - b.z)

    def __mul__(self, b):
        return vec3(self.x * b, self.y * b, self.z * b)

    __rmul__ = __mul__

    def __truediv__(self, b):
        inv = 1.0 / b
        return vec3(self.x * inv, self.y * inv, self.z * inv)

    def dot(self, b) -> float:
        return self.x * b.x + self.y * b.y + self.z * b.z

    def cross(self, b):
        return vec3(self.y * b.z - self.z * b.y, -(self.x * b.z - self.z * b.x), self.x * b.y - self.y * b.x)

    def distance(self, b) -> float:
        dx = b.x - self.x
        dy = b.y - self.y
        dz = b.z - self.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def length(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self):
        mag_inv = 1.0 / self.length()
        return vec3(self.x * mag_inv, self.y * mag_inv, self.z * mag_inv)
