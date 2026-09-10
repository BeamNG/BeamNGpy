from __future__ import annotations

from time import sleep

import pytest

from beamngpy import BeamNGpy
from beamngpy.sensors import Ultrasonic

import tests.sensors.helpers as h

USS_POS = (0, -2.3, 0.6)


def check_poll(bng: BeamNGpy, uss: Ultrasonic, is_auto: bool, exp_dist: float | None = None):
    distances: list[float] = []

    for i in range(h.ATTEMPTS):
        start, step = h.USS_OBSTACLE_START, h.USS_OBSTACLE_STEP_Y
        h.place_other_vehicle(
            bng, (start[0], start[1] + i * step, start[2]),
            settle_s=h.SETTLE_AFTER_TELEPORT_S,
        )
        attempt = i + 1
        if not is_auto or attempt > 1:
            sleep(0.5)

        if is_auto and attempt == 1:
            sensor_readings = h.wait_until_poll_valid(bng, uss, lambda r: r["distance"] < 100.0)
        else:
            sensor_readings = h.poll_auto_or_adhoc(uss, is_auto, attempt=attempt)

        obstacle_y = h.USS_OBSTACLE_START[1] + i * h.USS_OBSTACLE_STEP_Y
        print("Obstacle Y: ", obstacle_y)
        print("Expected distance (approx.): ", exp_dist)
        print("Ultrasonic readings: ", sensor_readings)
        assert len(sensor_readings) == 3
        for reading in sensor_readings.values():
            assert reading > 0

        distance = sensor_readings["distance"]
        distances.append(distance)
        if exp_dist and attempt == 1:
            assert abs(distance - exp_dist) <= 0.12, "Readings inaccurate"

    h.assert_responds_to_motion(distances, min_delta=0.03, label="ultrasonic distance")


ultrasonic_scene = h.module_scene(
    "ultrasonic_test",
    "Testing the ultrasonic sensor",
    "Ultrasonic test",
    obstacle_pos=h.USS_OBSTACLE_START,
)


@pytest.mark.parametrize("is_auto", [True, False])
def test_ultrasonic_poll(ultrasonic_scene, is_auto):
    """Automatic and ad-hoc polling return accurate distance readings."""
    bng, vehicle = ultrasonic_scene
    with h.using_sensor(
        Ultrasonic("ultrasonic1", bng, vehicle, pos=USS_POS),
        "Ultrasonic sensor removed.",
    ) as ultrasonic:
        check_poll(bng, ultrasonic, is_auto, 3.33)


def test_ultrasonic_negative_update(ultrasonic_scene):
    """A negative requested update time must not produce readings."""
    bng, vehicle = ultrasonic_scene
    with h.using_sensor(
        Ultrasonic("ultrasonic2", bng, vehicle, requested_update_time=-1.0, pos=USS_POS),
    ) as ultrasonic:
        print("Testing an ultrasonic sensor with a negative requested update time...")
        h.assert_negative_update(ultrasonic, lambda r: all(v >= 9999 for v in r.values()))


def test_ultrasonic_property_roundtrips(ultrasonic_scene):
    """Assert getter/setter round-trips for ultrasonic pose, scheduling, and radius."""
    bng, vehicle = ultrasonic_scene
    with h.using_sensor(Ultrasonic("ultrasonic_props", bng, vehicle)) as ultrasonic:
        h.exercise_roundtrip(
            ultrasonic,
            h.scheduling_pose_cases(ultrasonic, (-1.0, -2.0, 0.8)),
        )
        ultrasonic.set_is_visualised(False)
        assert ultrasonic.get_is_visualised() is False
        radius = ultrasonic.get_radius_at_distance(1.0)
        assert radius > 0.0
        h.assert_float_close(ultrasonic.get_radius_at_distance(1.0), radius)
        print(
            "Test visibility mode.  Ultrasonic visibility should cycle between on and off 3 times, staying at each for 1 second."
        )
        h.cycle_visibility(ultrasonic)
