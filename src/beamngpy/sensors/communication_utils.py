from __future__ import annotations

from beamngpy.connection import Connection
from beamngpy.types import StrDict


def send_sensor_request(connection: Connection, type: str, ack: str | None = None, **kwargs) -> StrDict:
    # Populate a dictionary with the data needed for a request from this sensor.
    data = dict(type=type, **kwargs)

    # Send the request, then wait for response from the simulation. NOTE: THIS BLOCKS EXECUTION HERE.
    response = connection.send(data)
    result = response.recv()
    if ack:
        response.ack(ack)
    return result


def set_sensor(connection: Connection, type: str, ack: str | None = None, **kwargs):
    # Populate a dictionary with the data needed for a request from this sensor.
    data = dict(type=type, **kwargs)

    # Send the request. If there is an acknowledge, check that it is the correct one.
    response = connection.send(data)
    if ack:
        response.ack(ack)
