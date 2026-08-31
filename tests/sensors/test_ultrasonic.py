from __future__ import annotations

from contextlib import contextmanager
from time import sleep
from typing import Iterator

import pytest

from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import Ultrasonic

from tests.sensors.helpers import (
    cycle_visibility,
    loaded_scenario,
    make_module_scene_fixture,
    poll_auto_or_adhoc,
    using_sensor,
)

ATTEMPTS = 3
USS_POS = (0, -2.3, 0.6)

def check_poll(uss: Ultrasonic, is_auto: bool, exp_dist: float | None = None):
    all_readings = []
    for i in range(1, ATTEMPTS + 1):
        sleep(2)
        sensor_readings = poll_auto_or_adhoc(uss, is_auto, attempt=i)
        print("Expected distance (approx.): ", exp_dist)
        print("Ultrasonic readings: ", sensor_readings)
        assert len(sensor_readings) == 3
        for reading in sensor_readings.values():
            assert reading > 0
        if exp_dist:
            assert abs(sensor_readings["distance"] - exp_dist) <= 0.1, "Readings inaccurate"
        all_readings.append(sensor_readings)

    for reading in ["distance", "windowMin", "windowMax"]:
        for i in range(1, ATTEMPTS - 1):
            assert all_readings[0][reading] != all_readings[i][reading], "Readings don't get updated"

        assert (
            abs(max(tmp[reading] for tmp in all_readings) - min(tmp[reading] for tmp in all_readings))
            <= 0.1
        ), "Readings inconsistent"

@contextmanager
def ultrasonic_scenario(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    vehicle = Vehicle("ego_vehicle", model="etki", licence="PYTHON", color="Red")
    obstacle = Vehicle("obstacle_vehicle", model="etki", licence="PYTHON", color="Blue")
    with loaded_scenario(
        beamng,
        level="tech_ground",
        name="ultrasonic_test",
        description="Testing the ultrasonic sensor",
        vehicles=[(vehicle, {}), (obstacle, {"pos": (0, -5, 0)})],
        label="Ultrasonic test",
    ) as scene:
        yield scene

ultrasonic_scene = make_module_scene_fixture(ultrasonic_scenario)

@pytest.mark.parametrize("is_auto", [True, False])
def test_ultrasonic_poll(ultrasonic_scene, is_auto):
    """Automatic and ad-hoc polling return accurate distance readings."""
    bng, vehicle = ultrasonic_scene
    with using_sensor(
        Ultrasonic("ultrasonic1", bng, vehicle, pos=USS_POS),
        "Ultrasonic sensor removed.",
    ) as ultrasonic:
        sleep(2)
        check_poll(ultrasonic, is_auto, 0.3)

def test_ultrasonic_negative_update(ultrasonic_scene):
    """A negative requested update time must not produce readings."""
    bng, vehicle = ultrasonic_scene
    with using_sensor(
        Ultrasonic("ultrasonic2", bng, vehicle, requested_update_time=-1.0, pos=USS_POS),
    ) as ultrasonic:
        print("Testing an ultrasonic sensor with a negative requested update time...")
        sleep(2)
        sensor_readings = ultrasonic.poll()
        print("Ultrasonic readings (should be zeros): ", sensor_readings)
        for reading in sensor_readings.values():
            assert reading >= 9999

def test_ultrasonic_properties(ultrasonic_scene):
    """Property getters, setters, and visibility mode switching."""
    bng, vehicle = ultrasonic_scene
    with using_sensor(Ultrasonic("ultrasonic1", bng, vehicle)) as ultrasonic:
        sleep(1)
        print(
            "Property getter test.  The displayed values should be the values which were set during the creation of the ultrasonic sensors."
        )
        print("Sensor Name: ", ultrasonic.name)
        print("Position: ", ultrasonic.get_position())
        print("Direction: ", ultrasonic.get_direction())
        print("Requested update time: ", ultrasonic.get_requested_update_time())
        print("Priority: ", ultrasonic.get_update_priority())
        print("Max Pending Requests: ", ultrasonic.get_max_pending_requests())
        print("Is Visualised [Flag]: ", ultrasonic.get_is_visualised())

        sleep(1)
        print(
            "Property setter test.  The displayed property values should be different from the previous values."
        )
        ultrasonic.set_requested_update_time(0.3)
        print("Newly-set Requested Update Time: ", ultrasonic.get_requested_update_time())
        ultrasonic.set_update_priority(0.5)
        print("Newly-set Priority: ", ultrasonic.get_update_priority())
        ultrasonic.set_max_pending_requests(5)
        print("Newly-set Max Pending Requests: ", ultrasonic.get_max_pending_requests())

        print(
            "Test visibility mode.  Ultrasonic visibility should cycle between on and off 3 times, staying at each for 1 second."
        )
        cycle_visibility(ultrasonic)
