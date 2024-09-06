from .sensor import Sensor


class GForces(Sensor):
    """
    This sensor is used to obtain the GForces acting on a vehicle.

    # TODO: GForce sensor for specific points on/in the vehicle
    """

    def __init__(self):
        super().__init__()

    def encode_vehicle_request(self):
        req = dict(type="GForces")
        return req
