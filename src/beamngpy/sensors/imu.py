from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from ..beamngcommon import BNGValueError
from .sensor import Sensor

if TYPE_CHECKING:
    from ..types import ConnData, Float3


class IMU(Sensor):
    """
    An IMU measures forces and rotational acceleration at a certain point on a
    vehicle. This can be used to analyze forces acting on certain areas of the
    car (like the driver's position) or estimate the trajectory of a vehicle
    from its rotation and acceleration.
    """

    def __init__(self, pos: Optional[Float3] = None, node=None, name: Optional[str] = None, debug=False):
        super().__init__()

        if pos is not None and node is not None:
            raise BNGValueError('Cannot specify both position and node for '
                                'an IMU')
        if pos is None and node is None:
            raise BNGValueError('Either position or node have to be specified '
                                'for an IMU')

        self._pos = pos
        self._node = node

        self._name = name if name is not None else str(hash(self))
        self._debug = debug

    def connect(self, bng, vehicle):
        if self._pos is not None:
            vehicle.add_imu_position(self._name, self._pos, self._debug)

        if self._node is not None:
            vehicle.add_imu_node(self._name, self._node, self._debug)

    def disconnect(self, bng, vehicle):
        vehicle.remove_imu(self._name)

    def encode_vehicle_request(self):
        req: ConnData = dict(type='IMU')
        req['name'] = self._name
        return req
