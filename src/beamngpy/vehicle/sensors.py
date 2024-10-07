from __future__ import annotations

from typing import TYPE_CHECKING, Dict, Iterable, Tuple

from beamngpy.logging import BNGValueError
from beamngpy.sensors import Sensor
from beamngpy.types import StrDict

if TYPE_CHECKING:
    from beamngpy.vehicle import Vehicle


class Sensors:
    """
    A sensor collection for a vehicle.

    Args:
        vehicle: The vehicle to which this object instance should belong to.
    """

    def __init__(self, vehicle: Vehicle):
        self._sensors: Dict[str, Sensor] = {}
        self.vehicle = vehicle

    @property
    def data(self):
        return self._sensors

    def attach(self, name: str, sensor: Sensor) -> None:
        """
        Enters a sensor into this vehicle's map of known sensors and calls the
        attach-hook of said sensor. The sensor is identified using the given
        name, which has to be unique among the other sensors of the vehicle.

        Args:
            name: The name of the sensor.
            sensor: The sensor to attach to the vehicle.
        """
        if name in self._sensors.keys():
            raise BNGValueError(
                'One vehicle cannot have multiple sensors with the same name: "{name}"'
            )
        self._sensors[name] = sensor
        sensor.attach(self.vehicle, name)

    def detach(self, name: str) -> None:
        """
        Detaches a sensor from the vehicle's map of known sensors and calls the detach-hook of said sensor.

        Args:
            name: The name of the sensor to disconnect.
        """
        if name in self._sensors:
            self._sensors[name].detach(self.vehicle, name)
        del self._sensors[name]

    def _encode_requests(self, sensor_names: Iterable[str]) -> Tuple[StrDict, StrDict]:
        """
        Encodes engine and vehicle requests for this vehicle's sensors and
        returns them as a tuple of (engine requests, vehicle requests).

        Args:
            sensor_names: List of sensor names to poll.

        Returns:
            A tuple of two lists: the engine requests and the vehicle requests
            to send to the simulation.
        """
        engine_reqs = dict()
        vehicle_reqs = dict()
        for name in sensor_names:
            sensor = self._sensors[name]
            engine_req = sensor.encode_engine_request()
            vehicle_req = sensor.encode_vehicle_request()

            if engine_req:
                engine_req["vehicle"] = self.vehicle.vid
                engine_reqs[name] = engine_req
            if vehicle_req:
                vehicle_reqs[name] = vehicle_req

        engine_reqs = dict(type="SensorRequest", sensors=engine_reqs)
        vehicle_reqs = dict(type="SensorRequest", sensors=vehicle_reqs)
        return engine_reqs, vehicle_reqs

    def _decode_response(self, sensor_data: StrDict) -> StrDict:
        """
        Goes over the given map of sensor data and decodes each of them iff
        they have a corresponding sensor to handle the data in this vehicle.
        The given map of sensor data is expected to have an entries that match
        up with sensor names in this vehicle.

        Args:
            sensor_data: The sensor data to decode as a dictionary,
                         identifying which sensor to decode data with by
                         the name it is known under in this vehicle.

        Returns:
            The decoded data as a dictionary with entries for each sensor name
            and corresponding decoded data.
        """
        response = dict()
        for name, data in sensor_data.items():
            sensor = self._sensors[name]
            data = sensor.decode_response(data)
            response[name] = data
        return response

    def poll(self, *sensor_names: str) -> None:
        """
        Updates the vehicle's sensor readings.

        Args:
            sensor_names: Names of sensors to poll. If none are provided, then all attached sensors
                          are polled.

        Returns:
            Nothing. Use ``vehicle.sensors[<sensor_id>][<data_access_id>]`` to
            access the polled sensor data.
        """
        if not sensor_names:
            sensor_names = tuple(self._sensors.keys())

        engine_reqs, vehicle_reqs = self._encode_requests(sensor_names)
        sensor_data = dict()

        engine_resp = (
            self.vehicle.bng._send(engine_reqs) if engine_reqs["sensors"] else None
        )
        vehicle_resp = (
            self.vehicle._send(vehicle_reqs) if vehicle_reqs["sensors"] else None
        )

        if engine_resp:
            resp = engine_resp.recv("SensorData")
            sensor_data.update(resp["data"])

        if vehicle_resp:
            resp = vehicle_resp.recv("SensorData")
            sensor_data.update(resp["data"])
        result = self._decode_response(sensor_data)

        for sensor, data in result.items():
            self._sensors[sensor].replace(data)

    def __repr__(self):
        return f"Sensors[{self.vehicle.vid}]: {self._sensors}"

    def __getitem__(self, __key: str) -> Sensor:
        return self._sensors.__getitem__(__key)

    def items(self):
        return self._sensors.items()
