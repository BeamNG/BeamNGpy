from __future__ import annotations

from contextlib import contextmanager
from time import sleep
from typing import Iterator

import numpy as np
import pytest

from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import AdvancedIMU

from tests.sensors.helpers import (
    cycle_visibility,
    latest_sensor_reading,
    loaded_scenario,
    make_module_scene_fixture,
    poll_auto_or_adhoc,
    using_sensor,
)

ATTEMPTS = 3

def check_consistency_and_update(imu: AdvancedIMU, is_auto: bool):
    all_readings = []

    for i in range(1, ATTEMPTS + 1):
        sleep(2)
        sensor_readings = latest_sensor_reading(poll_auto_or_adhoc(imu, is_auto, attempt=i))
        print(f"Advanced IMU readings: {sensor_readings}")

        assert (
            len(sensor_readings.keys()) > 0
            and "accSmooth" in sensor_readings
            and sensor_readings["accSmooth"][0] != 0
        ), "Readings not present"
        print("PASS: Readings present")
        all_readings.append(sensor_readings["accSmooth"][0])

    readings_diff = np.abs(np.max(all_readings) - np.min(all_readings))
    assert readings_diff < 0.01, f"Readings inconsistent: {readings_diff}"
    print(f"PASS: Readings consistent: {readings_diff}")

    assert np.any(
        (all_readings[0] != all_readings[1]) | (all_readings[0] != all_readings[2])
    ), "Readings don't get updated"
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

@contextmanager
def advanced_imu_scenario(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    vehicle = Vehicle("ego_vehicle", model="etk800", license="PYTHON", color="Red")
    with loaded_scenario(
        beamng,
        level="smallgrid",
        name="advanced_IMU_test",
        description="Testing the advanced IMU sensor",
        vehicles=[(vehicle, {})],
        label="Advanced IMU test",
    ) as scene:
        yield scene

advanced_imu_scene = make_module_scene_fixture(advanced_imu_scenario)

@pytest.mark.parametrize("is_auto", [True, False])
def test_advanced_imu_poll(advanced_imu_scene, is_auto):
    """Automatic and ad-hoc polling return consistent, updating readings."""
    bng, vehicle = advanced_imu_scene
    with using_sensor(_default_imu(bng, vehicle), "advanced IMU sensor removed.") as imu:
        sleep(10)
        check_consistency_and_update(imu, is_auto)

def test_advanced_imu_properties(advanced_imu_scene):
    """Property getters and visibility mode switching."""
    bng, vehicle = advanced_imu_scene
    with using_sensor(
        AdvancedIMU("advancedIMU1", bng, vehicle),
        "advanced IMU sensor removed.",
    ) as imu:
        sleep(1)
        print(
            "Property getter test.  The displayed values should be the values which were set during the creation of the advanced IMU sensor."
        )
        print(f"Sensor Name: {imu.name}")
        assert imu.name == "advancedIMU1"

        print(
            "Test visibility mode.  Advanced IMU visibility should cycle between on and off 3 times, staying at each for 1 second."
        )
        cycle_visibility(imu)

def test_advanced_imu_acceleration(advanced_imu_scene):
    """Vehicle acceleration is reflected in sensor readings."""
    bng, vehicle = advanced_imu_scene
    with using_sensor(_default_imu(bng, vehicle), "advanced IMU sensor removed.") as imu:
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
