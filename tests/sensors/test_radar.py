from __future__ import annotations

from time import sleep

import numpy as np
import pytest

from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import Radar

import tests.sensors.helpers as h

_RADAR = {
    "range_min": 0.1,
    "range_max": 100.0,
    "resolution": (200, 200),
    "fov": 70,
}


def _clip_radar_readings_to_common_length(all_readings: list[np.ndarray]) -> None:
    ref_len = all_readings[0].shape[0]
    for i in range(1, len(all_readings)):
        other_len = all_readings[i].shape[0]
        if np.abs(other_len - ref_len) >= ref_len * 0.05:
            raise AssertionError("Radar readings inconsistent")
        if other_len > ref_len:
            all_readings[i] = all_readings[i][:ref_len, :]
        elif other_len < ref_len:
            ref_len = other_len
            for j in range(i):
                all_readings[j] = all_readings[j][:ref_len, :]


def check_consistency_and_update(radar: Radar, test_name: str, is_auto: bool):
    all_readings = []

    for i in range(1, h.ATTEMPTS + 1):
        sleep(1)
        sensor_readings = h.poll_auto_or_adhoc(radar, is_auto, attempt=i)

        assert len(sensor_readings) > 0, "Readings not present"
        h.save_radar_plot(
            radar, test_name, i, sensor_readings,
            _RADAR["resolution"], _RADAR["fov"], (_RADAR["range_min"], _RADAR["range_max"]),
        )
        if not h.VISUALISE:
            print("RADAR readings: ", sensor_readings[0:10])
        print("PASS: Readings present")
        all_readings.append(sensor_readings)

    _clip_radar_readings_to_common_length(all_readings)

    print("PASS: Readings consistent")
    h.assert_polls_change(all_readings)
    print("PASS: Readings get updated")


def _full_config_radar(bng: BeamNGpy, vehicle: Vehicle) -> Radar:
    return Radar(
        "radar1", bng, vehicle,
        requested_update_time=0.01,
        pos=(0, 0, 1.7), dir=h.SENSOR_TEST_DIR, up=h.SENSOR_TEST_UP,
        size=_RADAR["resolution"], field_of_view_y=_RADAR["fov"],
        near_far_planes=(_RADAR["range_min"], _RADAR["range_max"]),
        range_roundness=-2.0, range_cutoff_sensitivity=0.0,
        range_shape=0.23, range_focus=0.12,
        range_min_cutoff=0.5, range_direct_max_cutoff=_RADAR["range_max"],
    )


radar_scene = h.module_scene(
    "radar_test", "Testing the RADAR sensor", "RADAR test",
    obstacle_pos=h.OBSTACLE_START,
)


@pytest.mark.parametrize("is_auto", [True, False])
def test_radar_poll(radar_scene, request, is_auto):
    """Automatic and ad-hoc polling return consistent, updating readings."""
    bng, vehicle = radar_scene
    with h.using_sensor(_full_config_radar(bng, vehicle), "RADAR sensor removed.") as radar:
        check_consistency_and_update(radar, request.node.name, is_auto)


def test_radar_negative_update(radar_scene):
    """A negative requested update time must not produce readings."""
    bng, vehicle = radar_scene
    with h.using_sensor(Radar("radar2", bng, vehicle, requested_update_time=-1.0)) as radar:
        print("Testing a RADAR sensor with a negative requested update time...")
        h.assert_negative_update(radar, lambda r: r.shape[0] == 0)


def test_radar_property_roundtrips(radar_scene):
    """Assert getter/setter round-trips for RADAR pose and scheduling."""
    bng, vehicle = radar_scene
    with h.using_sensor(Radar("radar_props", bng, vehicle)) as radar:
        assert radar.name == "radar_props"
        h.assert_float_close(radar.get_requested_update_time(), 0.1)
        h.assert_float_close(radar.get_update_priority(), 0.0)
        assert radar.get_max_pending_requests() == 10
        h.exercise_roundtrip(
            radar,
            h.scheduling_pose_cases(radar, (-1.0, 0.0, 1.8)),
        )
