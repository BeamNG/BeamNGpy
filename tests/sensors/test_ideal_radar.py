from __future__ import annotations

from beamngpy.sensors import IdealRadar

from tests.sensors.helpers import OBSTACLE_START, exercise_roundtrip, module_scene, using_sensor

ideal_radar_scene = module_scene(
    "ideal_radar_test", "Testing the ideal RADAR sensor", "Ideal RADAR test",
    obstacle_pos=OBSTACLE_START,
)


def test_ideal_radar_property_roundtrips(ideal_radar_scene):
    """Assert getter/setter round-trip for ideal RADAR update time."""
    bng, vehicle = ideal_radar_scene
    with using_sensor(
        IdealRadar("ideal_radar1", bng, vehicle, is_send_immediately=False, physics_update_time=0.01),
        "ideal RADAR sensor removed.",
    ) as ideal_radar:
        exercise_roundtrip(ideal_radar, update_time=0.01)
