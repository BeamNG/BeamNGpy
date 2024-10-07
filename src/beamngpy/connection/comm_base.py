from __future__ import annotations

from typing import TYPE_CHECKING

from beamngpy.logging import BNGDisconnectedError, create_warning
from beamngpy.types import Any, StrDict

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.connection import Connection
    from beamngpy.vehicle import Vehicle


class CommBase:
    """
    Communication helper base class to make the socket communication easier to implement for derived classes.
    """

    @staticmethod
    def _send_recv(connection: Connection, type: str, **kwargs) -> StrDict:
        """
        Sends data to the specified connection, receives the answer and returns it.
        """
        # Populate a dictionary with the data needed for a request from this sensor.
        data = dict(type=type, **kwargs)

        # Send the request, then wait for response from the simulation. NOTE: THIS BLOCKS EXECUTION HERE.
        response = connection.send(data)
        result = response.recv()
        return result

    @staticmethod
    def _send_ack(connection: Connection, type: str, ack: str, **kwargs) -> None:
        """
        Sends data to the specified connection, and receives the acknowledgement.
        """
        # Populate a dictionary with the data needed for a request from this sensor.
        data = dict(type=type, **kwargs)

        # Send the request. If there is an acknowledge, check that it is the correct one.
        response = connection.send(data)
        if ack:
            response.ack(ack)

    def __init__(self, bng: BeamNGpy, vehicle: Vehicle | None):
        self.bng = bng
        self.vehicle = vehicle

    def send_recv_ge(self, type: str, **kwargs: Any) -> StrDict:
        """
        Sends a request to the GE Lua with the provided type and data, receives the
        answer and returns it.

        Args:
            type: Type of the request to send.
            kwargs: The other data being sent.

        Returns:
            The response of the simulator.
        """
        if kwargs.get("ack"):
            create_warning(
                "You are sending an `ack` argument to the `send_recv_ge` function, "
                "which does not waits for acknowledgements. You may be looking for "
                "the `send_ack_ge` function."
            )

        if not self.bng.connection:
            raise BNGDisconnectedError("The simulator is not connected!")
        return CommBase._send_recv(self.bng.connection, type, **kwargs)

    def send_ack_ge(self, type: str, ack: str, **kwargs: Any) -> None:
        """
        Sends a request to the GE Lua with the provided type and data, and
        receives the acknowledgement.

        Args:
            type: Type of the request to send.
            ack: Type of the acknowledgement to be received.
            kwargs: The other data being sent.

        Returns:
            The response of the simulator.
        """
        if not self.bng.connection:
            raise BNGDisconnectedError("The simulator is not connected!")
        CommBase._send_ack(self.bng.connection, type, ack=ack, **kwargs)

    def send_recv_veh(self, type: str, **kwargs: Any) -> StrDict:
        """
        Sends a request to the Vehicle Lua with the provided type and data, receives the
        answer and returns it.

        Args:
            type: Type of the request to send.
            kwargs: The other data being sent.

        Returns:
            The response of the simulator.
        """
        if kwargs.get("ack"):
            create_warning(
                "You are sending an `ack` argument to the `send_recv_veh` function, "
                "which does not waits for acknowledgements. You may be looking for "
                "the `send_ack_veh` function."
            )

        if not self.vehicle or not self.vehicle.connection:
            raise BNGDisconnectedError("The vehicle is not connected!")
        return CommBase._send_recv(self.vehicle.connection, type, **kwargs)

    def send_ack_veh(self, type: str, ack: str, **kwargs: Any) -> None:
        """
        Sends a request to the Vehicle Lua with the provided type and data, and
        receives the acknowledgement.

        Args:
            type: Type of the request to send.
            ack: Type of the acknowledgement to be received.
            kwargs: The other data being sent.

        Returns:
            The response of the simulator.
        """
        if not self.vehicle or not self.vehicle.connection:
            raise BNGDisconnectedError("The vehicle is not connected!")
        CommBase._send_ack(self.vehicle.connection, type, ack=ack, **kwargs)
