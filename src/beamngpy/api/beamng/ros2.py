from __future__ import annotations

from typing import Any

from .base import Api
from beamngpy.types import StrDict
from beamngpy.vehicle import Vehicle


class Ros2Api(Api):
    """
    An API for controlling the BeamNG ROS 2 bridge from BeamNGpy.

    This API configures and starts the ROS 2 extension in BeamNG.tech. It does
    not stream ROS 2 sensor data back through BeamNGpy.

    Args:
        beamng: An instance of the simulator.
    """

    def __init__(self, beamng):
        super().__init__(beamng)
        self._extension_loaded = False

    def _ensure_extension(self) -> None:
        if not self._extension_loaded:
            self._beamng.control.queue_lua_command(
                "if not extensions.isExtensionLoaded('tech_ros_pythonApi') "
                "then extensions.load('tech_ros_pythonApi') end"
            )
            self._extension_loaded = True

    def _recv(self, type: str, **kwargs: Any) -> StrDict:
        self._ensure_extension()
        return self._send(dict(type=type, **kwargs)).recv(type)

    def _ack(self, type: str, **kwargs: Any) -> None:
        self._ensure_extension()
        self._send(dict(type=type, **kwargs)).ack(type)

    @staticmethod
    def _vid(vehicle: Vehicle | str) -> str:
        return vehicle.vid if isinstance(vehicle, Vehicle) else vehicle

    def _vehicle_ack(self, message_type: str, vehicle: Vehicle | str, **kwargs: Any) -> None:
        self._ack(message_type, vid=self._vid(vehicle), **kwargs)

    def _vehicle_recv(self, message_type: str, vehicle: Vehicle | str, **kwargs: Any) -> StrDict:
        return self._recv(message_type, vid=self._vid(vehicle), **kwargs)

    def load_bridge(self, path: str | None = None) -> None:
        """
        Load the ROS 2 bridge library.

        Args:
            path: Full path or bare library filename. If omitted, BeamNG.tech
                reuses the last path selected in the editor when available.
        """
        self._ack("Ros2LoadBridge", path=path)

    def get_bridge_info(self) -> StrDict:
        """
        Get bridge load state and path.

        Returns:
            A dictionary with ``loaded`` and ``path``.
        """
        resp = self._recv("Ros2GetBridgeInfo")
        return {"loaded": resp["loaded"], "path": resp.get("path")}

    def start(self, vehicle: Vehicle | str) -> None:
        """
        Start or restart the ROS 2 node for a vehicle.

        Args:
            vehicle: Vehicle object or vehicle id/name.
        """
        self._vehicle_ack("Ros2Start", vehicle)

    def start_with_config(self, vehicle: Vehicle | str, config: StrDict) -> None:
        """
        Apply a launch config and start the ROS 2 node for a vehicle.

        Args:
            vehicle: Vehicle object or vehicle id/name.
            config: ROS 2 launch config table.
        """
        self._vehicle_ack("Ros2StartWithConfig", vehicle, config=config)

    def stop(self, vehicle: Vehicle | str) -> None:
        """
        Stop the ROS 2 node for a vehicle.

        Args:
            vehicle: Vehicle object or vehicle id/name.
        """
        self._vehicle_ack("Ros2Stop", vehicle)

    def start_all(self) -> int:
        """
        Start every vehicle that has configured sensors.

        Returns:
            The number of vehicles started.
        """
        return self._recv("Ros2StartAll")["count"]

    def stop_all(self) -> int:
        """
        Stop all active ROS 2 vehicle nodes.

        Returns:
            The number of stopped nodes.
        """
        return self._recv("Ros2StopAll")["count"]

    def add_sensor(
        self,
        vehicle: Vehicle | str,
        sensor_type: str,
        config: StrDict | None = None,
        fields: list[str] | None = None,
        position: list[float] | None = None,
        direction: list[float] | None = None,
        up: list[float] | None = None,
    ) -> str:
        """
        Add a ROS 2 sensor config to a vehicle.

        Args:
            vehicle: Vehicle object or vehicle id/name.
            sensor_type: Sensor type from :meth:`list_sensor_types`.
            config: Values merged over the defaults for this type.
            fields: Scalar field names to publish for scalar sensors.
            position: Vehicle-space position coefficients.
            direction: Vehicle-space forward direction coefficients.
            up: Vehicle-space up direction coefficients.

        Returns:
            The generated sensor id, for example ``Electrics_1``.
        """
        opts = {
            key: value
            for key, value in (
                ("config", config),
                ("fields", fields),
                ("position", position),
                ("direction", direction),
                ("up", up),
            )
            if value is not None
        }
        return self._vehicle_recv(
            "Ros2AddSensor", vehicle, sensorType=sensor_type, opts=opts
        )["sensorId"]

    def remove_sensor(self, vehicle: Vehicle | str, sensor_id: str) -> None:
        """
        Remove one configured sensor from a vehicle.

        Args:
            vehicle: Vehicle object or vehicle id/name.
            sensor_id: Sensor id returned by :meth:`add_sensor`.
        """
        self._vehicle_ack("Ros2RemoveSensor", vehicle, sensorId=sensor_id)

    def clear_sensors(self, vehicle: Vehicle | str) -> None:
        """
        Remove all configured ROS 2 sensors from a vehicle.

        Args:
            vehicle: Vehicle object or vehicle id/name.
        """
        self._vehicle_ack("Ros2ClearSensors", vehicle)

    def list_sensors(self, vehicle: Vehicle | str) -> StrDict:
        """
        List the stored ROS 2 sensor config for a vehicle.

        Args:
            vehicle: Vehicle object or vehicle id/name.

        Returns:
            The stored draft config table.
        """
        return self._vehicle_recv("Ros2ListSensors", vehicle)["sensors"]

    def set_sensor_field(
        self, vehicle: Vehicle | str, sensor_id: str, path: str, value: Any
    ) -> None:
        """
        Set a dotted sensor config field.

        Args:
            vehicle: Vehicle object or vehicle id/name.
            sensor_id: Sensor id.
            path: Dotted config path, for example ``config.create.pos``.
            value: Value to store.
        """
        self._vehicle_ack(
            "Ros2SetSensorField", vehicle, sensorId=sensor_id, path=path, value=value
        )

    def get_sensor_field(self, vehicle: Vehicle | str, sensor_id: str, path: str) -> Any:
        """
        Read a dotted sensor config field.

        Args:
            vehicle: Vehicle object or vehicle id/name.
            sensor_id: Sensor id.
            path: Dotted config path.

        Returns:
            The stored value, or ``None`` when the path does not exist.
        """
        return self._vehicle_recv(
            "Ros2GetSensorField", vehicle, sensorId=sensor_id, path=path
        ).get("value")

    def set_sensor_fields(
        self, vehicle: Vehicle | str, sensor_id: str, fields: list[str] | None
    ) -> None:
        """
        Restrict a scalar sensor to selected fields, or restore all fields.

        Args:
            vehicle: Vehicle object or vehicle id/name.
            sensor_id: Sensor id.
            fields: Field names to publish, or ``None`` to publish the full catalog.
        """
        self._vehicle_ack("Ros2SetSensorFields", vehicle, sensorId=sensor_id, fields=fields)

    def set_update_rate(self, vehicle: Vehicle | str, sensor_id: str, hz: float) -> None:
        """
        Set ``config.update_rate_hz`` for a sensor.

        Args:
            vehicle: Vehicle object or vehicle id/name.
            sensor_id: Sensor id.
            hz: Publish rate in Hz.
        """
        self._vehicle_ack("Ros2SetUpdateRate", vehicle, sensorId=sensor_id, hz=hz)

    def set_publish_mode(
        self, vehicle: Vehicle | str, sensor_id: str, mode: str
    ) -> None:
        """
        Set a sensor publish mode.

        Args:
            vehicle: Vehicle object or vehicle id/name.
            sensor_id: Sensor id.
            mode: ``fixed`` or ``max``.
        """
        self._vehicle_ack("Ros2SetPublishMode", vehicle, sensorId=sensor_id, mode=mode)

    def list_sensor_types(self) -> list[str]:
        """
        List sensor types known to the ROS 2 extension.

        Returns:
            Sensor type names from ``rosDefaults.json``.
        """
        return self._recv("Ros2ListSensorTypes")["types"]

    def get_sensor_schema(self, sensor_type: str) -> StrDict | None:
        """
        Get config keys and scalar field catalog for one sensor type.

        Args:
            sensor_type: Sensor type name.

        Returns:
            The type schema, or ``None`` when the type is unknown.
        """
        return self._recv("Ros2GetSensorSchema", sensorType=sensor_type).get("schema")

    def apply_config(self, vehicle: Vehicle | str, config: StrDict) -> None:
        """
        Store a complete ROS 2 launch config without starting the node.

        Args:
            vehicle: Vehicle object or vehicle id/name.
            config: ROS 2 launch config table.
        """
        self._vehicle_ack("Ros2ApplyConfig", vehicle, config=config)

    def get_config(self, vehicle: Vehicle | str) -> StrDict:
        """
        Get the stored config merged with ``rosDefaults.json``.

        Args:
            vehicle: Vehicle object or vehicle id/name.
        """
        return self._vehicle_recv("Ros2GetConfig", vehicle)["config"]

    def get_live_config(self, vehicle: Vehicle | str) -> StrDict | None:
        """
        Get the config currently published by a running node.

        Args:
            vehicle: Vehicle object or vehicle id/name.

        Returns:
            The live config, or ``None`` when the node is not running.
        """
        return self._vehicle_recv("Ros2GetLiveConfig", vehicle).get("config")

    def save_config(self, vehicle: Vehicle | str, path: str) -> None:
        """
        Save a vehicle's config to disk on the BeamNG.tech host.

        Args:
            vehicle: Vehicle object or vehicle id/name.
            path: Host-side path to write.
        """
        self._vehicle_ack("Ros2SaveConfig", vehicle, path=path)

    def load_config(self, vehicle: Vehicle | str, path: str) -> StrDict:
        """
        Load a vehicle config from disk on the BeamNG.tech host.

        Args:
            vehicle: Vehicle object or vehicle id/name.
            path: Host-side path to read.

        Returns:
            The normalized config after loading.
        """
        return self._vehicle_recv("Ros2LoadConfig", vehicle, path=path)["config"]

    def get_status(self, vehicle: Vehicle | str) -> StrDict:
        """
        Get ROS 2 status for a vehicle.

        Args:
            vehicle: Vehicle object or vehicle id/name.
        """
        return self._vehicle_recv("Ros2GetStatus", vehicle)["status"]

    def validate_config(self, vehicle: Vehicle | str) -> list[str]:
        """
        Validate a vehicle's stored ROS 2 config.

        Args:
            vehicle: Vehicle object or vehicle id/name.

        Returns:
            A list of human-readable problems. An empty list means the config can launch.
        """
        return self._vehicle_recv("Ros2ValidateConfig", vehicle)["problems"]

    def list_vehicles(self) -> list[StrDict]:
        """
        List scene vehicles with ROS 2 status summary fields.

        Returns:
            Vehicle rows containing name, jbeam, ROS active state and sensor count.
        """
        return self._recv("Ros2ListVehicles")["vehicles"]

    def is_node_active(self, vehicle: Vehicle | str) -> bool:
        """
        Check whether the vehicle's ROS 2 node is publishing.
        """
        return self.get_status(vehicle)["nodeActive"]

    def is_stopping(self, vehicle: Vehicle | str) -> bool:
        """
        Check whether a vehicle node teardown is still pending.
        """
        return self.get_status(vehicle)["stopping"]

    def get_node_name(self, vehicle: Vehicle | str) -> str:
        """
        Get the ROS 2 node name for a vehicle.
        """
        return self.get_status(vehicle)["nodeName"]

    def get_topic_name(self, vehicle: Vehicle | str, sensor_id: str) -> str:
        """
        Get a ROS 2 topic name for a vehicle sensor id.
        """
        status = self.get_status(vehicle)
        return f"{status['nodeNamespace']}/{sensor_id}"
