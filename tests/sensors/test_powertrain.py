from __future__ import annotations

from time import sleep

import pytest

from beamngpy.sensors import PowertrainSensor

from tests.helpers import exercise_roundtrip, module_scene, poll_auto_or_adhoc, using_sensor

powertrain_scene = module_scene("powertrain_test", "Testing the powertrain sensor", "Powertrain test")


@pytest.mark.parametrize("is_auto", [True, False], ids=["automatic", "ad_hoc"])
def test_powertrain_poll(powertrain_scene, is_auto):
    """Automatic and ad-hoc polling return powertrain readings."""
    bng, vehicle = powertrain_scene
    with using_sensor(
        PowertrainSensor("powertrain1", bng, vehicle, is_send_immediately=False, physics_update_time=0.001),
        "powertrain sensor removed.",
    ) as powertrain:
        sleep(1)
        sensor_readings = poll_auto_or_adhoc(powertrain, is_auto)
        print("powertrain readings: ", sensor_readings)


def test_powertrain_property_roundtrips(powertrain_scene):
    """Assert getter/setter round-trip for powertrain update time."""
    bng, vehicle = powertrain_scene
    with using_sensor(
        PowertrainSensor("powertrain1", bng, vehicle, is_send_immediately=False, physics_update_time=0.001),
        "powertrain sensor removed.",
    ) as powertrain:
        exercise_roundtrip(powertrain, update_time=0.001)
