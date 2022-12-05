from __future__ import annotations

from typing import TYPE_CHECKING

from beamngpy.logging import BNGValueError
from beamngpy.types import Float3, StrDict

from .sensor import Sensor

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle


class IMU(Sensor):
    """
    An IMU measures forces and rotational acceleration at a certain point on a
    vehicle. This can be used to analyze forces acting on certain areas of the
    car (like the driver's position) or estimate the trajectory of a vehicle
    from its rotation and acceleration.
    """

    def __init__(self, pos: Float3 | None = None, node: int | None = None, name: str | None = None, debug: bool = False):
        super().__init__()

        if pos is not None and node is not None:
            raise BNGValueError('Cannot specify both position and node for an IMU')
        if pos is None and node is None:
            raise BNGValueError('Either position or node have to be specified for an IMU')

        self._pos = pos
        self._node = node

        self._name = name if name is not None else str(id(self))
        self._debug = debug

    def connect(self, bng: BeamNGpy, vehicle: Vehicle) -> None:
        if self._pos is not None:
            IMU._add_imu_position(vehicle, self._name, self._pos, self._debug)

        if self._node is not None:
            IMU._add_imu_node(vehicle, self._name, self._node, self._debug)

    def disconnect(self, bng: BeamNGpy, vehicle: Vehicle) -> None:
        IMU._remove_imu(vehicle, self._name)

    def encode_vehicle_request(self):
        req: StrDict = dict(type='IMU')
        req['name'] = self._name
        return req

    @staticmethod
    def _add_imu_position(vehicle: Vehicle, name: str, pos: Float3, debug: bool = False):
        """
        Adds an IMU to this vehicle at the given position identified by the
        given name. The position is relative to the vehicle's coordinate
        system, meaning (0, 0, 0) will always refer to the vehicle's origin
        regardless of its position in the world. This is to make addition of
        IMUs independent of the vehicle spawn position. To find an appropriate
        position relative to the vehicle's origin, it's recommended to inspect
        the vehicle's nodes in the vehicle editor ingame and retrieve the
        original relative positions of nodes close to the desired measurement
        point.

        Args:
            name: The name this IMU is identified by. This is mainly
                        used to later remove an IMU.
            pos: The measurement point relative to the vehicle's origin.
            debug: Optional flag which enables debug rendering of the
                          IMU. Useful to verify placement.
        """
        data: StrDict = dict(type='AddIMUPosition')
        data['name'] = name
        data['pos'] = pos
        data['debug'] = debug
        vehicle._send(data).ack('IMUPositionAdded')

    @staticmethod
    def _add_imu_node(vehicle: Vehicle, name: str, node: int, debug: bool = False):
        """
        Adds an IMU to this vehicle at the given node identified by the given
        name. The node is specified as a number and can be found by inspecting
        the vehicle using the ingame vehicle editor.

        Args:
            name: The name this IMU is identified by. This is mainly used to later remove an IMU.
            node: The node ID to perform measurements at.
            debug: Optional flag which enables debug rendering of the IMU.
                   Useful to verify placement.
        """
        data: StrDict = dict(type='AddIMUNode')
        data['name'] = name
        data['node'] = node
        data['debug'] = debug
        vehicle._send(data).ack('IMUNodeAdded')

    @staticmethod
    def _remove_imu(vehicle: Vehicle, name: str):
        """
        Removes the IMU identified by the given name.

        Args:
            name: The name of the IMU to be removed.

        Raises:
            BNGValueError: If there is no IMU with the specified name.
        """
        data = dict(type='RemoveIMU')
        data['name'] = name
        vehicle._send(data).ack('IMURemoved')
