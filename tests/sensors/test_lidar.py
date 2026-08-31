from __future__ import annotations

from contextlib import contextmanager
from time import sleep
from typing import Iterator

import numpy as np
import pytest

from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import Lidar

from tests.sensors.helpers import (
    cycle_visibility,
    loaded_scenario,
    make_module_scene_fixture,
    poll_auto_or_adhoc,
    using_sensor,
)

ATTEMPTS = 3

def validate_distribution(pointCloud):
    x, y = pointCloud[:, 0], pointCloud[:, 1]
    phi = np.degrees(np.arctan2(y, x))
    counts, _ = np.histogram(phi, bins=16, range=(-180, 180))
    return np.max(counts) - np.min(counts) < np.max(counts) / 5

def polling_check(lidar: Lidar, is_auto: bool, is_360: bool, is_rotate: bool):
    data_total = []
    prev_first_point = None
    data = None

    for i in range(ATTEMPTS):
        sleep(2)
        data = poll_auto_or_adhoc(lidar, is_auto, attempt=i + 1)

        print("LiDAR point cloud data: ", data["pointCloud"])
        print("LiDAR colour data: ", data["colours"])

        assert len(data["pointCloud"]) > 0
        print("PASS: Point cloud is present")

        if is_360:
            assert validate_distribution(data["pointCloud"]), "360 point cloud data is not uniform."
            print("PASS: Data is uniform")

        if i == 0:
            prev_first_point = data["pointCloud"][0]
        if not is_rotate:
            data_total.append(len(data["pointCloud"]))
        assert len(data["colours"]) > 0
        print("PASS: Colour data is present")

    assert prev_first_point is not None and (data["pointCloud"][0] != prev_first_point).all(), "Point cloud doesn't get updated"
    print("PASS: Point cloud gets updated")

    if not is_rotate:
        assert np.max(data_total) - np.min(data_total) < np.max(data_total) * 0.01, "360 or static point cloud data is inconsistent"
    print("PASS: LiDAR point cloud data is consistent")

def negative_update_test(lidar: Lidar):
    sleep(2)
    data = lidar.poll()
    print("LiDAR point cloud data (should be zeros): ", data["pointCloud"])
    print("LiDAR colour data (should be zeros): ", data["colours"])
    assert len(data["pointCloud"]) == 0 and len(data["colours"]) == 0
    print("PASS: No data is present")

@contextmanager
def lidar_scenario(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    vehicle = Vehicle("ego_vehicle", model="etki", license="PYTHON", color="Blue")
    with loaded_scenario(
        beamng,
        level="tech_ground",
        name="lidar_test",
        description="Testing the LiDAR sensor",
        vehicles=[(vehicle, {"pos": (0, 0, 0)})],
        label="LiDAR test",
    ) as scene:
        yield scene

lidar_scene = make_module_scene_fixture(lidar_scenario)

@pytest.mark.parametrize("is_using_shared_memory", [True, False])
def test_lidar_shmem(lidar_scene, is_using_shared_memory):
    """Default 360 mode with and without shared memory."""
    bng, vehicle = lidar_scene
    label = "shared memory" if is_using_shared_memory else "socket (no shared memory)"
    print(f"Testing a LiDAR sensor which uses {label}...")
    with using_sensor(
        Lidar("lidar1", bng, vehicle, is_using_shared_memory=is_using_shared_memory),
        "LiDAR sensor removed.",
    ) as lidar:
        polling_check(lidar, True, True, False)
        polling_check(lidar, False, True, False)

@pytest.mark.parametrize("is_using_shared_memory", [True, False])
def test_lidar_negative_update(lidar_scene, is_using_shared_memory):
    """Negative requested update time must not produce readings."""
    bng, vehicle = lidar_scene
    mem = "WITH" if is_using_shared_memory else "WITHOUT"
    print(f"Testing a LiDAR sensor with a negative requested update time ({mem} shared memory)...")
    with using_sensor(
        Lidar(
            "lidar3",
            bng,
            vehicle,
            requested_update_time=-1.0,
            is_using_shared_memory=is_using_shared_memory,
        ),
        "LiDAR sensor removed.",
    ) as lidar:
        negative_update_test(lidar)

def test_lidar_lfo_mode(lidar_scene):
    """LFO (rotate) mode with a 90 degree aperture."""
    bng, vehicle = lidar_scene
    print("Testing a LiDAR sensor in LFO mode...")
    with using_sensor(
        Lidar(
            "lidar5", bng, vehicle,
            is_using_shared_memory=True, is_360_mode=False,
            is_rotate_mode=True, frequency=1, horizontal_angle=90,
        ),
        "LiDAR sensor removed.",
    ) as lidar:
        polling_check(lidar, True, False, True)
        polling_check(lidar, False, False, True)

def test_lidar_static_mode(lidar_scene):
    """Static mode with a 90 degree aperture."""
    bng, vehicle = lidar_scene
    print("Testing a LiDAR sensor in static mode...")
    with using_sensor(
        Lidar(
            "lidar6", bng, vehicle,
            is_using_shared_memory=True, is_360_mode=False, horizontal_angle=90,
        ),
        "LiDAR sensor removed.",
    ) as lidar:
        polling_check(lidar, True, False, False)
        polling_check(lidar, False, False, False)

def test_lidar_properties(lidar_scene):
    """Property getters and setters, visualisation and annotation mode switching."""
    bng, vehicle = lidar_scene
    with using_sensor(Lidar("lidar1", bng, vehicle), "LiDAR sensor removed.") as lidar:
        sleep(1)
        print(
            "Property getter test.  The displayed values should be the values which were set during the creation of the LiDAR sensors."
        )
        print("Sensor Name: ", lidar.name)
        print("Position: ", lidar.get_position())
        print("Direction: ", lidar.get_direction())
        print("Requested update time: ", lidar.get_requested_update_time())
        print("Priority: ", lidar.get_update_priority())
        print("Max Pending Requests: ", lidar.get_max_pending_requests())
        print("Is Visualised [Flag]: ", lidar.get_is_visualised())
        print("Is Annotated [Flag]: ", lidar.get_is_annotated())

        sleep(1)
        print(
            "Property setter test.  The displayed property values should be different from the previous values."
        )
        lidar.set_requested_update_time(0.3)
        print("Newly-set Requested Update Time: ", lidar.get_requested_update_time())
        lidar.set_update_priority(0.5)
        print("Newly-set Priority: ", lidar.get_update_priority())
        lidar.set_max_pending_requests(5)
        print("Newly-set Max Pending Requests: ", lidar.get_max_pending_requests())

        print(
            "Test visibility mode.  LiDAR visibility should cycle between on and off 3 times, staying at each for 1 second."
        )
        cycle_visibility(lidar)

        print(
            "Test LiDAR mode.  LiDAR should cycle between DEPTH MODE and ANNOTATION MODE, staying at each for 1 second."
        )
        for i in range(6):
            sleep(1)
            lidar.set_is_annotated(not (i % 2 == 0))
