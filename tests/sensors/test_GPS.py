from __future__ import annotations

from beamngpy.sensors import GPS

from tests.sensors.helpers import exercise_roundtrip, module_scene, using_sensor

gps_scene = module_scene("gps_test", "Testing the GPS sensor", "GPS test")


def test_gps_property_roundtrips(gps_scene):
    """Assert getter/setter round-trips for GPS update time and visualisation."""
    bng, vehicle = gps_scene
    with using_sensor(
        GPS("gps1", bng, vehicle, is_send_immediately=False, physics_update_time=0.001, is_visualised=True),
        "GPS sensor removed.",
    ) as gps:
        exercise_roundtrip(gps, update_time=0.001)
        gps.set_is_visualised(False)
        assert gps.get_is_visualised() is False
