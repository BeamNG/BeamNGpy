from __future__ import annotations

from beamngpy.types import StrDict

from .sensor import Sensor


class Damage(Sensor):
    """
    The damage sensor retrieves information about how damaged the structure
    of the vehicle is. It's important to realise that this is a sensor that has
    no analogue in real life as it returns a perfect knowledge overview of how
    deformed the vehicle is. It's therefore more of a ground truth than
    simulated sensor data.
    """

    def __init__(self):
        super().__init__()

    def encode_vehicle_request(self):
        req: StrDict = dict(type="Damage")
        if "part_damage" in req.keys():
            req["part_damage"] = req["part_damage"] if req["part_damage"] else {}
        return req
