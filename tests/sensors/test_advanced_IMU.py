from __future__ import annotations

from time import sleep
import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import AdvancedIMU

def check_consistency_and_update(imu: AdvancedIMU, is_auto: bool):
    all_readings = []
    attempts = 3

    # Make sure we retrieve actively updated and consistent readings.
    for i in range(1, attempts + 1):
        sleep(2)
        if is_auto:
            print("\nAutomatic polling attempt " + str(i))
            sensor_readings = imu.poll()
        else:
            print("Ad-hoc poll request attempt " + str(i))
            request_id = (
                imu.send_ad_hoc_poll_request()
            )  # send an ad-hoc polling request to the simulator.
            print("Ad-hoc poll requests sent. Unique request Id number: ", request_id)
            sleep(3)
            print(
                "Is ad-hoc request complete? ",
                imu.is_ad_hoc_poll_request_ready(request_id),
            )  # Ensure that the data has been processed before collecting.
            sensor_readings = imu.collect_ad_hoc_poll_request(
                request_id
            )  # Collect the data now that it has been computed.

        print("Advanced IMU readings: ", sensor_readings)
        if not isinstance(sensor_readings, list): sensor_readings = [sensor_readings]
        assert len(sensor_readings) > 0 \
            and "accSmooth" in sensor_readings[-1] \
            and sensor_readings[-1]["accSmooth"][0] != 0, "Readings not present"
        print("PASS: Readings present")
        all_readings.append(sensor_readings[-1]["accSmooth"][0])

    assert np.abs(np.max(all_readings) - np.min(all_readings)) < 0.8, "Readings inconsistent"
    print("PASS: Readings consistent")

    assert np.any((all_readings[0] != all_readings[1]) | (all_readings[0] != all_readings[2])), "Readings don't get updated"
    print("PASS: Readings get updated")

def test_advanced_IMU(beamng: BeamNGpy):
    with beamng as bng:
        vehicle = Vehicle(
            "ego_vehicle", model="etki", license="PYTHON", color="Red"
        )  # Create a vehicle.
        scenario = Scenario(
            "tech_ground",
            "advanced_IMU_test",
            description="Testing the advanced IMU sensor",
        )  # Create a scenario.
        scenario.add_vehicle(vehicle)  # Add the vehicle to the scenario.
        scenario.make(bng)
        bng.settings.set_deterministic(60)  # Set simulator to 60hz temporal resolution
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        print("Advanced IMU test start.")

        # Create a default advanced IMU sensor.
        IMU1 = AdvancedIMU("advancedIMU1", bng, vehicle, is_send_immediately=True)

        # Test the automatic polling functionality of the advanced IMU sensor, to make sure we retrieve the readings data via shared memory.
        sleep(5)
        check_consistency_and_update(IMU1, True)

        # Test the ad-hoc polling functionality of the advanced IMU sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
        print("Ad-hoc poll request test.")
        check_consistency_and_update(IMU1, False)

        # Test vehicle acceleration is represented in the data
        print("Testing vehicle acceleration can be seen in the sensor readings")
        vehicle.control(throttle=100)
        sleep(3)
        sensor_readings = IMU1.poll()
        if isinstance(sensor_readings, list): sensor_readings = sensor_readings[-1]
        assert sensor_readings["accSmooth"][0] > 3

        IMU1.remove()
        print("advanced IMU sensor removed.")

        # Recreate the advanced IMU sensor.
        IMU1 = AdvancedIMU("advancedIMU1", bng, vehicle)

        # Test that the property getter function return the correct data which was set.
        sleep(1)
        print(
            "Property getter test.  The displayed values should be the values which were set during the creation of the advanced IMU sensor."
        )
        print("Sensor Name: ", IMU1.name)
        assert IMU1.name == "advancedIMU1"

        # Test changing the visibility of the sensor.
        print(
            "Test visibility mode.  Advanced IMU visibility should cycle between on and off 3 times, staying at each for 1 second."
        )
        sleep(1)
        IMU1.set_is_visualised(False)
        sleep(1)
        IMU1.set_is_visualised(True)
        sleep(1)
        IMU1.set_is_visualised(False)
        sleep(1)
        IMU1.set_is_visualised(True)
        sleep(1)
        IMU1.set_is_visualised(False)
        sleep(1)
        IMU1.set_is_visualised(True)

        IMU1.remove()

        sleep(3)
        print("advanced IMU test complete.")
        bng.ui.show_hud()


# Executing this file will perform various tests on all available functionality relating to the advanced IMU sensor.
# It is provided to give examples on how to use all advanced IMU sensor functions currently available in beamngpy.
if __name__ == "__main__":
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    test_advanced_IMU(bng)
