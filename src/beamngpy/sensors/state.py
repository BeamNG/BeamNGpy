from .sensor import Sensor


class State(Sensor):
    """
    The state sensor monitors general stats of the vehicle, such as position,
    direction, velocity, etc.

    It contains the following:
        * ``time``: The current simulation time in seconds.
        * ``pos``: The vehicle's position as an ``(x, y, z)`` triplet
        * ``dir``: The vehicle's direction vector as an ``(x, y, z)`` triplet
        * ``up``: The vehicle's up vector as an ``(x, y, z)`` triplet
        * ``vel``: The vehicle's velocity along each axis in metres per second as an ``(x, y, z)`` triplet
        * ``rotation``: The vehicle's rotation as an ``(x, y, z, w)`` quaternion
    """

    def __init__(self):
        super().__init__()
        self.connected = False

    def connect(self, bng, vehicle):
        self.connected = True

    def disconnect(self, bng, vehicle):
        self.connected = False

    def encode_vehicle_request(self):
        req = dict(type="State")
        return req

    def decode_response(self, resp):
        if "state" in resp:
            return resp["state"]

        return None
