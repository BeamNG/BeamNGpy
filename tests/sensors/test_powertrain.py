from __future__ import annotations

from contextlib import contextmanager
from time import sleep
from typing import Iterator

import pytest

from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import PowertrainSensor

from tests.sensors.helpers import (
    loaded_scenario,
    make_module_scene_fixture,
    poll_auto_or_adhoc,
    using_sensor,
)

def _default_powertrain(bng: BeamNGpy, vehicle: Vehicle) -> PowertrainSensor:
    return PowertrainSensor(
        "powertrain1", bng, vehicle,
        is_send_immediately=False,
        physics_update_time=0.001,
    )

@contextmanager
def powertrain_scenario(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    vehicle = Vehicle("ego_vehicle", model="etki", license="PYTHON", color="Red")
    with loaded_scenario(
        beamng,
        level="tech_ground",
        name="powertrain_test",
        description="Testing the powertrain sensor",
        vehicles=[(vehicle, {})],
        label="Powertrain test",
    ) as scene:
        yield scene

powertrain_scene = make_module_scene_fixture(powertrain_scenario)

@pytest.mark.parametrize("is_auto", [True, False])
def test_powertrain_poll(powertrain_scene, is_auto):
    """Automatic and ad-hoc polling return powertrain readings."""
    bng, vehicle = powertrain_scene
    with using_sensor(_default_powertrain(bng, vehicle), "powertrain sensor removed.") as powertrain:
        sleep(2)
        if is_auto:
            sensor_readings = powertrain.poll()
            print("powertrain readings (automatic polling): ", sensor_readings)
        else:
            print("Ad-hoc poll request test.")
            sensor_readings = poll_auto_or_adhoc(powertrain, False)
            print("powertrain readings (ad-hoc polling): ", sensor_readings)
