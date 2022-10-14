from typing import Optional

from ..connection import Connection


def _send_sensor_request(connection: Connection, type: str, ack: Optional[str] = None, **kwargs):
    # Populate a dictionary with the data needed for a request from this sensor.
    data = dict(type=type, **kwargs)
    # Send the request for updated readings to the simulation, receive the updated readings from the simulation.
    response = connection.send(data)
    result = response.recv()
    if ack:
        response.ack(ack)
    return result


def _set_sensor(connection: Connection, type: str, **kwargs):
    # Populate a dictionary with the data needed for a request from this sensor.
    data = dict(type=type, **kwargs)
    # Send the request for updated readings to the simulation.
    return connection.send(data)
