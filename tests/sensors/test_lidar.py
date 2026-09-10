from __future__ import annotations

from time import sleep

import numpy as np
import pytest

from beamngpy import BeamNGpy
from beamngpy.sensors import Lidar

import tests.sensors.helpers as h


def validate_distribution(pointCloud):
    x, y = pointCloud[:, 0], pointCloud[:, 1]
    phi = np.degrees(np.arctan2(y, x))
    counts, _ = np.histogram(phi, bins=16, range=(-180, 180))
    return np.max(counts) - np.min(counts) < np.max(counts) / 5


def _cloud_signature(point_cloud: np.ndarray) -> float:
    finite = point_cloud[np.isfinite(point_cloud).all(axis=1)]
    valid = finite[(np.abs(finite) < 1e6).all(axis=1)]
    return float(np.sum(np.round(valid, 1)))


def polling_check(
    bng: BeamNGpy, lidar: Lidar, is_auto: bool, is_360: bool, is_rotate: bool,
):
    signatures: list[float] = []
    update_min_delta = 1.0 if is_rotate else 250.0

    for i in range(h.ATTEMPTS):
        if not is_rotate:
            start, step = h.OBSTACLE_START, h.LIDAR_OBSTACLE_STEP_Y
            h.place_other_vehicle(
                bng, (start[0], start[1] + i * step, start[2]),
                settle_s=h.SETTLE_AFTER_TELEPORT_S,
            )
        else:
            sleep(2)

        data = h.poll_auto_or_adhoc(lidar, is_auto, attempt=i + 1)

        print("LiDAR point cloud data: ", data["pointCloud"])
        print("LiDAR colour data: ", data["colours"])

        assert len(data["pointCloud"]) > 0
        print("PASS: Point cloud is present")

        if is_360 and i == 0:
            assert validate_distribution(data["pointCloud"]), "360 point cloud data is not uniform."
            print("PASS: Data is uniform")

        signatures.append(_cloud_signature(data["pointCloud"]))
        assert len(data["colours"]) > 0
        print("PASS: Colour data is present")

    h.assert_responds_to_motion(signatures, min_delta=update_min_delta, label="lidar cloud signature")
    print("PASS: Point cloud reflects scene changes")

    h.assert_duplicate_poll_consistent(lidar, is_auto, lambda d: len(d["pointCloud"]))
    print("PASS: LiDAR point cloud data is consistent")


lidar_scene = h.module_scene(
    "lidar_test",
    "Testing the LiDAR sensor",
    "LiDAR test",
    obstacle_pos=h.OBSTACLE_START,
)


@pytest.mark.parametrize("is_using_shared_memory", [True, False])
@pytest.mark.parametrize("is_auto", [True, False])
def test_lidar_shmem(lidar_scene, is_using_shared_memory, is_auto):
    """Default 360 mode with and without shared memory."""
    bng, vehicle = lidar_scene
    label = "shared memory" if is_using_shared_memory else "socket (no shared memory)"
    print(f"Testing a LiDAR sensor which uses {label}...")
    with h.using_sensor(
        Lidar("lidar1", bng, vehicle, is_using_shared_memory=is_using_shared_memory),
        "LiDAR sensor removed.",
    ) as lidar:
        polling_check(bng, lidar, is_auto, True, False)


@pytest.mark.parametrize("is_using_shared_memory", [True, False])
def test_lidar_negative_update(lidar_scene, is_using_shared_memory):
    """Negative requested update time must not produce readings."""
    bng, vehicle = lidar_scene
    mem = "WITH" if is_using_shared_memory else "WITHOUT"
    print(f"Testing a LiDAR sensor with a negative requested update time ({mem} shared memory)...")
    with h.using_sensor(
        Lidar(
            "lidar3",
            bng,
            vehicle,
            requested_update_time=-1.0,
            is_using_shared_memory=is_using_shared_memory,
        ),
        "LiDAR sensor removed.",
    ) as lidar:
        h.assert_negative_update(
            lidar,
            lambda d: len(d["pointCloud"]) == 0 and len(d["colours"]) == 0,
        )
        print("PASS: No data is present")


@pytest.mark.parametrize("is_auto", [True, False])
def test_lidar_lfo_mode(lidar_scene, is_auto):
    """LFO (rotate) mode with a 90 degree aperture."""
    bng, vehicle = lidar_scene
    print("Testing a LiDAR sensor in LFO mode...")
    with h.using_sensor(
        Lidar(
            "lidar5", bng, vehicle,
            is_using_shared_memory=True, is_360_mode=False,
            is_rotate_mode=True, frequency=1, horizontal_angle=90,
        ),
        "LiDAR sensor removed.",
    ) as lidar:
        polling_check(bng, lidar, is_auto, False, True)


@pytest.mark.parametrize("is_auto", [True, False])
def test_lidar_static_mode(lidar_scene, is_auto):
    """Static mode with a 90 degree aperture."""
    bng, vehicle = lidar_scene
    print("Testing a LiDAR sensor in static mode...")
    with h.using_sensor(
        Lidar(
            "lidar6", bng, vehicle,
            is_using_shared_memory=True, is_360_mode=False, horizontal_angle=90,
        ),
        "LiDAR sensor removed.",
    ) as lidar:
        polling_check(bng, lidar, is_auto, False, False)


def test_lidar_property_roundtrips(lidar_scene):
    """Assert getter/setter round-trips for LiDAR pose, tuning, and scheduling."""
    bng, vehicle = lidar_scene
    with h.using_sensor(Lidar("lidar_props", bng, vehicle), "LiDAR sensor removed.") as lidar:
        h.exercise_roundtrip(
            lidar,
            h.scheduling_pose_cases(lidar, (-2.0, 0.5, 1.5), extra=[
                ("get_vertical_resolution", 32, lidar.set_vertical_resolution),
                ("get_frequency", 15.0, lidar.set_frequency),
                ("get_max_distance", 90.0, lidar.set_max_distance),
            ]),
        )
        lidar.set_is_visualised(False)
        assert lidar.get_is_visualised() is False
        lidar.set_is_annotated(True)
        assert lidar.get_is_annotated() is True
        print(
            "Test visibility mode.  LiDAR visibility should cycle between on and off 3 times, staying at each for 1 second."
        )
        h.cycle_visibility(lidar)
        if h.VISUALISE:
            print(
                "Test LiDAR mode.  LiDAR should cycle between DEPTH MODE and ANNOTATION MODE, staying at each for 1 second."
            )
            for i in range(6):
                sleep(1)
                lidar.set_is_annotated(not (i % 2 == 0))
