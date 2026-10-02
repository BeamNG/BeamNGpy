"""Socket contract for beamngpy.api.beamng.ros2.Ros2Api.

Run from the beamngpy repo:

    python -m unittest discover -s tests -p test_ros2_api.py
"""

from __future__ import annotations

import os
import re
import unittest
from pathlib import Path

from beamngpy.api.beamng.ros2 import Ros2Api


REPLY = {
    "loaded": True,
    "path": "ros2_windows_humble.dll",
    "count": 1,
    "sensorId": "Electrics_1",
    "sensors": {"sensors": []},
    "value": 3,
    "types": ["Electrics"],
    "schema": {"type": "Electrics"},
    "config": {"sensors": []},
    "status": {
        "nodeActive": True,
        "stopping": False,
        "nodeName": "bng1",
        "nodeNamespace": "/bng1",
    },
    "problems": [],
    "vehicles": [],
}

VEHICLE_REQUESTS = {
    "Ros2Start",
    "Ros2StartWithConfig",
    "Ros2Stop",
    "Ros2AddSensor",
    "Ros2RemoveSensor",
    "Ros2ClearSensors",
    "Ros2ListSensors",
    "Ros2SetSensorField",
    "Ros2GetSensorField",
    "Ros2SetSensorFields",
    "Ros2SetUpdateRate",
    "Ros2SetPublishMode",
    "Ros2ApplyConfig",
    "Ros2GetConfig",
    "Ros2GetLiveConfig",
    "Ros2SaveConfig",
    "Ros2LoadConfig",
    "Ros2GetStatus",
    "Ros2ValidateConfig",
}


class _FakeBeamng:
    def __init__(self):
        self.sent = []
        self.control = self
        self._message = None
        self.logger = None

    def queue_lua_command(self, command):
        self.sent.append(command)

    def _send(self, data):
        self.sent.append(data)
        return self

    def ack(self, message_type):
        assert self.sent[-1]["type"] == message_type

    def recv(self, message_type):
        assert self.sent[-1]["type"] == message_type
        return dict(REPLY, type=message_type)


def _messages(fake):
    return [item for item in fake.sent if isinstance(item, dict)]


class Ros2ApiTest(unittest.TestCase):
    def setUp(self):
        self.fake = _FakeBeamng()
        self.api = Ros2Api(self.fake)

    def test_extension_loads_once(self):
        self.api.load_bridge("ros2_windows_humble")
        self.api.get_bridge_info()
        loads = [item for item in self.fake.sent if isinstance(item, str)]
        self.assertEqual(len(loads), 1)
        self.assertIn("tech_ros_pythonApi", loads[0])

    def test_add_sensor_omits_unset_options(self):
        sensor_id = self.api.add_sensor("ego", "Lidar", position=[0.5, 0, 1])
        self.assertEqual(sensor_id, "Electrics_1")
        message = _messages(self.fake)[-1]
        self.assertEqual(message["vid"], "ego")
        self.assertEqual(message["sensorType"], "Lidar")
        self.assertEqual(message["opts"], {"position": [0.5, 0, 1]})

    def test_topic_name_uses_status_namespace(self):
        self.assertEqual(self.api.get_topic_name("ego", "Lidar_1"), "/bng1/Lidar_1")
        self.assertTrue(self.api.is_node_active("ego"))
        self.assertFalse(self.api.is_stopping("ego"))
        self.assertEqual(self.api.get_node_name("ego"), "bng1")

    def test_socket_messages_match_lua_routes(self):
        api = self.api
        api.load_bridge(None)
        api.get_bridge_info()
        api.start("ego")
        api.start_with_config("ego", {"sensors": []})
        api.stop("ego")
        api.start_all()
        api.stop_all()
        api.add_sensor("ego", "Electrics", fields=["throttle"])
        api.remove_sensor("ego", "Electrics_1")
        api.clear_sensors("ego")
        api.list_sensors("ego")
        api.set_sensor_field("ego", "Electrics_1", "config.update_rate_hz", 10)
        api.get_sensor_field("ego", "Electrics_1", "config.update_rate_hz")
        api.set_sensor_fields("ego", "Electrics_1", None)
        api.set_update_rate("ego", "Electrics_1", 5)
        api.set_publish_mode("ego", "Electrics_1", "fixed")
        api.list_sensor_types()
        api.get_sensor_schema("Electrics")
        api.apply_config("ego", {"sensors": []})
        api.get_config("ego")
        api.get_live_config("ego")
        api.save_config("ego", "tech/ros.json")
        api.load_config("ego", "tech/ros.json")
        api.get_status("ego")
        api.validate_config("ego")
        api.list_vehicles()

        sent = {message["type"] for message in _messages(self.fake)}
        for message in _messages(self.fake):
            if message["type"] in VEHICLE_REQUESTS:
                self.assertIn("vid", message)
            else:
                self.assertNotIn("vid", message)

        lua_path = Path(os.environ.get("BEAMNG_GAME_ROOT", r"C:\game")) / (
            "lua/ge/extensions/tech/ros/pythonApi.lua"
        )
        if not lua_path.is_file():
            self.skipTest(f"game pythonApi.lua not found at {lua_path}")
        lua_source = lua_path.read_text(encoding="utf-8")
        lua_routes = set(re.findall(r"(?m)^\s*(Ros2[A-Za-z]+)\s*=", lua_source))
        lua_routes.update(re.findall(r"registerOne\('(Ros2[A-Za-z]+)'", lua_source))
        self.assertEqual(sent, lua_routes)

        def lua_block(start, end):
            return lua_source.split(start, 1)[1].split(end, 1)[0]

        lua_vehicle = set(re.findall(
            r"(?m)^\s*(Ros2[A-Za-z]+)\s*=",
            lua_block("local vehicleAck = {", "local plainAck")
            + lua_block("local vehicleQuery = {", "local plainQuery"),
        ))
        lua_vehicle.update(re.findall(r"registerOne\('(Ros2[A-Za-z]+)', true", lua_source))
        self.assertEqual(VEHICLE_REQUESTS, lua_vehicle)


if __name__ == "__main__":
    unittest.main()
