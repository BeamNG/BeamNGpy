from __future__ import annotations

from beamngpy.connection import Connection
from beamngpy.types import StrDict


def send_sensor_request(connection: Connection, type: str, ack: str | None = None, **kwargs) -> StrDict:
    # Populate a dictionary with the data needed for a request from this sensor.
    data = dict(type=type, **kwargs)
    # Send the request for updated readings to the simulation, receive the updated readings from the simulation.
    response = connection.send(data)
    result = response.recv()
    if ack:
        response.ack(ack)
    return result


def set_sensor(connection: Connection, type: str, ack: str | None = None, **kwargs):
    # Populate a dictionary with the data needed for a request from this sensor.
    data = dict(type=type, **kwargs)
    # Send the request for updated readings to the simulation.
    response = connection.send(data)
    if ack:
        response.ack(ack)
