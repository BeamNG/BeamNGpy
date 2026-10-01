from __future__ import annotations

from time import sleep

import numpy as np
import pytest

from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import AdvancedIMU

import tests.helpers as h


def check_consistency_and_update(imu: AdvancedIMU, is_auto: bool):
    all_readings = []

    for i in range(1, h.ATTEMPTS + 1):
        sleep(1)
        sensor_readings = h.poll_auto_or_adhoc(imu, is_auto, attempt=i)
        if isinstance(sensor_readings, list):
            reading = sensor_readings[-1]
        else:
            reading = sensor_readings[list(sensor_readings.keys())[-1]]
        print(f"Advanced IMU readings: {reading}")

        assert (
            len(reading.keys()) > 0
            and "accSmooth" in reading
            and reading["accSmooth"][0] != 0
        ), "Readings not present"
        print("PASS: Readings present")
        all_readings.append(reading["accSmooth"][0])

    spread = abs(max(all_readings) - min(all_readings))
    assert spread < 0.01, f"Readings inconsistent: {spread}"
    print("PASS: Readings consistent")
    h.assert_polls_change(all_readings)
    print("PASS: Readings get updated")


def _default_imu(bng: BeamNGpy, vehicle: Vehicle) -> AdvancedIMU:
    return AdvancedIMU(
        "imu1", bng, vehicle,
        pos=(0, 0, 5),
        physics_update_time=0.0005,
        gfx_update_time=0.0005,
        smoother_strength=1.0,
        is_using_gravity=True,
        is_visualised=True,
        is_snapping_desired=True,
        is_force_inside_triangle=False,
        is_allow_wheel_nodes=False,
    )


advanced_imu_scene = h.module_scene(
    "advanced_IMU_test", "Testing the advanced IMU sensor", "Advanced IMU test",
)


@pytest.fixture(autouse=True)
def _reset_shared_scene(advanced_imu_scene):
    bng, vehicle = advanced_imu_scene
    h.restart_vehicles(bng, vehicle, pause=False)


@pytest.mark.parametrize("is_auto", [True, False], ids=["automatic", "ad_hoc"])
def test_advanced_imu_poll(advanced_imu_scene, is_auto):
    """Automatic and ad-hoc polling return consistent, updating readings."""
    bng, vehicle = advanced_imu_scene
    with h.using_sensor(_default_imu(bng, vehicle), "advanced IMU sensor removed.") as imu:
        bng.control.step(600)
        check_consistency_and_update(imu, is_auto)


def test_advanced_imu_property_roundtrips(advanced_imu_scene):
    """Assert getter/setter round-trips for Advanced IMU scheduling and flags."""
    bng, vehicle = advanced_imu_scene
    with h.using_sensor(
        AdvancedIMU("advancedIMU_props", bng, vehicle),
        "advanced IMU sensor removed.",
    ) as imu:
        assert imu.name == "advancedIMU_props"
        h.cycle_visibility(imu)
        h.exercise_roundtrip(imu)
        imu.set_is_using_gravity(True)
        assert imu.get_is_using_gravity() is True
        imu.set_is_visualised(False)
        assert imu.get_is_visualised() is False


def test_advanced_imu_acceleration(advanced_imu_scene):
    """Vehicle acceleration is reflected in sensor readings."""
    bng, vehicle = advanced_imu_scene
    with h.using_sensor(_default_imu(bng, vehicle), "advanced IMU sensor removed.") as imu:
        print("Testing vehicle acceleration can be seen in the sensor readings")

        vehicle.control(throttle=0.2)
        imu.poll()
        vehicle.sensors.poll("state")
        while vehicle.state["vel"][1] > -11:
            vehicle.sensors.poll("state")
            sleep(0.1)

        sensor_readings = imu.poll()
        readings = sensor_readings if isinstance(sensor_readings, list) else list(sensor_readings.values())
        accel_values = np.array([reading["accSmooth"][0] for reading in readings])
        assert np.mean(accel_values) > 1.2 and np.mean(accel_values) < 1.5
        assert np.std(accel_values) < 0.7
        print("PASS: Vehicle acceleration can be seen in the sensor readings")
