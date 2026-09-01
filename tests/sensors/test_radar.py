from __future__ import annotations

from contextlib import contextmanager
from time import sleep
from typing import Iterator

import numpy as np
import pytest

from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import Radar

from tests.sensors.helpers import (
    loaded_scenario,
    make_module_scene_fixture,
    poll_auto_or_adhoc,
    using_sensor,
)

RANGE_MIN = 0.1
RANGE_MAX = 100.0
RESOLUTION = (200, 200)
FOV = 70
VISUALISE = False
ATTEMPTS = 3

def check_consistency_and_update(radar: Radar, is_auto: bool):
    all_readings = []

    for i in range(1, ATTEMPTS + 1):
        sleep(1)
        sensor_readings = poll_auto_or_adhoc(radar, is_auto, attempt=i)

        assert len(sensor_readings) > 0, "Readings not present"
        if VISUALISE:
            radar.plot_data(
                sensor_readings, RESOLUTION, FOV, RANGE_MIN, RANGE_MAX, RESOLUTION[0], RESOLUTION[1]
            )
        else:
            print("RADAR readings: ", sensor_readings[0:10])
        print("PASS: Readings present")
        all_readings.append(sensor_readings)

    # readings will often not match in length
    # if they have only a small difference, clip the longer to match the shorter
    for i in range(1, 3):
        if np.abs(all_readings[i].shape[0] - all_readings[0].shape[0]) < all_readings[0].shape[0] * 0.05:
            if all_readings[i].shape[0] - all_readings[0].shape[0] > 0:
                all_readings[i] = all_readings[i][:all_readings[0].shape[0], :]
            else:
                all_readings[0] = all_readings[0][:all_readings[i].shape[0], :]
                if i == 2:
                    all_readings[1] = all_readings[1][:all_readings[i].shape[0], :]
        else:
            raise AssertionError("Radar readings inconsistent")

    print("PASS: Readings consistent")

    assert np.any((all_readings[0] != all_readings[1]) | (all_readings[0] != all_readings[2])), "Readings don't get updated"
    print("PASS: Readings get updated")

def _full_config_radar(bng: BeamNGpy, vehicle: Vehicle) -> Radar:
    return Radar(
        "radar1", bng, vehicle,
        requested_update_time=0.01,
        pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1),
        size=RESOLUTION, field_of_view_y=FOV,
        near_far_planes=(RANGE_MIN, RANGE_MAX),
        range_roundness=-2.0, range_cutoff_sensitivity=0.0,
        range_shape=0.23, range_focus=0.12,
        range_min_cutoff=0.5, range_direct_max_cutoff=RANGE_MAX,
    )

@contextmanager
def radar_scenario(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    vehicle = Vehicle("ego_vehicle", model="etki", licence="PYTHON", color="Red")
    vehicle2 = Vehicle("other_vehicle", model="etki", licence="PYTHON", color="Blue")
    with loaded_scenario(
        beamng,
        level="smallgrid",
        name="radar_test",
        description="Testing the RADAR sensor",
        vehicles=[(vehicle, {}), (vehicle2, {"pos": (0, -10, 0)})],
        label="RADAR test",
    ) as scene:
        yield scene

radar_scene = make_module_scene_fixture(radar_scenario)

@pytest.mark.parametrize("is_auto", [True, False])
def test_radar_poll(radar_scene, is_auto):
    """Automatic and ad-hoc polling return consistent, updating readings."""
    bng, vehicle = radar_scene
    with using_sensor(_full_config_radar(bng, vehicle), "RADAR sensor removed.") as radar:
        check_consistency_and_update(radar, is_auto)

def test_radar_negative_update(radar_scene):
    """A negative requested update time must not produce readings."""
    bng, vehicle = radar_scene
    with using_sensor(Radar("radar2", bng, vehicle, requested_update_time=-1.0)) as radar:
        print("Testing an ultrasonic sensor with a negative requested update time...")
        sleep(1)
        sensor_readings = radar.poll()
        print("RADAR readings (should be zeros): ", sensor_readings)
        assert sensor_readings.shape[0] == 0

def test_radar_properties(radar_scene):
    """Property getters and setters with expected default values."""
    bng, vehicle = radar_scene
    with using_sensor(Radar("radar1", bng, vehicle)) as radar:
        print(
            "Property getter test.  The displayed values should be the values which were set during the creation of the ultrasonic sensors."
        )
        print("Sensor Name: ", radar.name)
        assert radar.name == "radar1"
        print("Position: ", radar.get_position())
        print("Direction: ", radar.get_direction())
        print("Requested update time: ", radar.get_requested_update_time())
        assert np.round(radar.get_requested_update_time(), 3) == 0.1
        print("Priority: ", radar.get_update_priority())
        assert np.round(radar.get_update_priority(), 3) == 0
        print("Max Pending Requests: ", radar.get_max_pending_requests())
        assert radar.get_max_pending_requests() == 10

        print(
            "Property setter test.  The displayed property values should be different from the previous values."
        )
        radar.set_requested_update_time(0.3)
        print("Newly-set Requested Update Time: ", radar.get_requested_update_time())
        assert np.round(radar.get_requested_update_time(), 3) == 0.3
        radar.set_update_priority(0.5)
        print("Newly-set Priority: ", radar.get_update_priority())
        assert np.round(radar.get_update_priority(), 3) == 0.5
        radar.set_max_pending_requests(5)
        print("Newly-set Max Pending Requests: ", radar.get_max_pending_requests())
        assert radar.get_max_pending_requests() == 5
