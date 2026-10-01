from __future__ import annotations

from beamngpy.sensors import RoadsSensor

from tests.helpers import exercise_roundtrip, module_scene, using_sensor

roads_sensor_scene = module_scene(
    "roads_sensor_test", "Testing the roads sensor", "Roads sensor test",
)


def test_roads_sensor_property_roundtrips(roads_sensor_scene):
    """Assert getter/setter round-trip for roads sensor update time."""
    bng, vehicle = roads_sensor_scene
    with using_sensor(
        RoadsSensor("roads1", bng, vehicle, is_send_immediately=False, physics_update_time=0.01, is_visualised=False),
        "roads sensor removed.",
    ) as roads_sensor:
        exercise_roundtrip(roads_sensor, update_time=0.01)
