from __future__ import annotations

from time import sleep
import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import AdvancedIMU

ATTEMPTS = 3

def check_consistency_and_update(imu: AdvancedIMU, is_auto: bool):
    all_readings = []

    # Make sure we retrieve actively updated and consistent readings.
    for i in range(1, ATTEMPTS + 1):
        sleep(2)
        if is_auto:
            print(f"\nAutomatic polling attempt {i}")
            sensor_readings = imu.poll()
        else:
            print(f"Ad-hoc poll request attempt {i}")
            # Send an ad-hoc polling request to the simulator.
            request_id = imu.send_ad_hoc_poll_request()
            print(f"Ad-hoc poll requests sent. Unique request Id number: {request_id}")
            sleep(3)
            # Ensure that the data has been processed before collecting.
            print(
                "Is ad-hoc request complete? ",
                imu.is_ad_hoc_poll_request_ready(request_id)
            )
            # Collect the data now that it has been computed.
            sensor_readings = imu.collect_ad_hoc_poll_request(request_id)

        if isinstance(sensor_readings, list):
            sensor_readings = sensor_readings[-1]
        else:
            sensor_readings = sensor_readings[list(sensor_readings.keys())[-1]]
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

def test_advanced_IMU(beamng: BeamNGpy):
    with beamng as bng:
        vehicle = Vehicle(
            "ego_vehicle", model="etk800", license="PYTHON", color="Red"
        )  # Create a vehicle.
        scenario = Scenario(
            "smallgrid",
            "advanced_IMU_test",
            description="Testing the advanced IMU sensor",
        )  # Create a scenario.
        scenario.add_vehicle(vehicle)  # Add the vehicle to the scenario.
        scenario.make(bng)
        bng.settings.set_deterministic(60)  # Set simulator to 60hz temporal resolution
        bng.scenario.load(scenario)
        bng.scenario.start()

        print("Advanced IMU test start.")

        # Create a default advanced IMU sensor.
        IMU1 = AdvancedIMU(
            "imu1", bng, vehicle,
            pos=(0, 0, 5),
            physics_update_time=0.0005,
            gfx_update_time=0.0005,
            smoother_strength=1.0,
            is_using_gravity=True,
            is_visualised=True,
            is_snapping_desired=True,
            is_force_inside_triangle=False,
            is_allow_wheel_nodes=False
        )

        # Test the automatic polling functionality of the advanced IMU sensor, to make sure we retrieve the readings data via shared memory.
        sleep(10)
        check_consistency_and_update(IMU1, True)

        # Test the ad-hoc polling functionality of the advanced IMU sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
        check_consistency_and_update(IMU1, False)

        # Test vehicle acceleration is represented in the data
        print("Testing vehicle acceleration can be seen in the sensor readings")

        vehicle.control(throttle=0.2)
        IMU1.poll()
        vehicle.sensors.poll('state')
        while vehicle.state['vel'][1] > -11:
            vehicle.sensors.poll('state')
            sleep(0.1)

        sensor_readings = IMU1.poll()
        if isinstance(sensor_readings, list):
            readings = sensor_readings
        else:
            readings = list(sensor_readings.values())
        accel_values = np.array([
            reading['accSmooth'][0] for reading in readings
        ])
        assert np.mean(accel_values) > 1.2 and np.mean(accel_values) < 1.5
        assert np.std(accel_values) < 0.7
        print("PASS: Vehicle acceleration can be seen in the sensor readings")

        IMU1.remove()
        print("advanced IMU sensor removed.")

        # Recreate the advanced IMU sensor.
        IMU1 = AdvancedIMU("advancedIMU1", bng, vehicle)

        # Test that the property getter function return the correct data which was set.
        sleep(1)
        print(
            "Property getter test.  The displayed values should be the values which were set during the creation of the advanced IMU sensor."
        )
        print(f"Sensor Name: {IMU1.name}")
        assert IMU1.name == "advancedIMU1"

        # Test changing the visibility of the sensor.
        print(
            "Test visibility mode.  Advanced IMU visibility should cycle between on and off 3 times, staying at each for 1 second."
        )
        for i in range(0, 6):
            sleep(1)
            IMU1.set_is_visualised(not (i % 2 == 0))

        IMU1.remove()

        sleep(3)
        print("advanced IMU test complete.")

# Executing this file will perform various tests on all available functionality relating to the advanced IMU sensor.
# It is provided to give examples on how to use all advanced IMU sensor functions currently available in beamngpy.
if __name__ == "__main__":
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252, quit_on_close=False)
    test_advanced_IMU(bng)
