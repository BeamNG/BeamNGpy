def _send_sensor_request(connection, type, ack=None, **kwargs):
    # Populate a dictionary with the data needed for a request from this sensor.
    data = dict(type=type, **kwargs)
    # Send the request for updated readings to the simulation.
    resp = connection.send(data)
    # Receive the updated readings from the simulation.
    result = resp.recv()
    if ack:
        resp.ack(ack)
    return result

def _set_sensor(connection, type, **kwargs):
    # Populate a dictionary with the data needed for a request from this sensor.
    data = dict(type=type, **kwargs)
    # Send the request for updated readings to the simulation.
    return connection.send(data)