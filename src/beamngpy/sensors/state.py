from .sensor import Sensor


class State(Sensor):
    """
    The state sensor monitors general stats of the vehicle, such as position,
    direction, velocity, etc. It is a default sensor every vehicle has and is
    used to update the vehicle.state attribute.
    """

    def __init__(self):
        super().__init__()
        self.connected = False

    def connect(self, bng, vehicle):
        self.connected = True

    def disconnect(self, bng, vehicle):
        self.connected = False

    def encode_vehicle_request(self):
        req = dict(type='State')
        return req

    def decode_response(self, resp):
        if 'state' in resp:
            return resp['state']

        return None
