from __future__ import annotations

from time import sleep
import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Ultrasonic

ATTEMPTS = 3

def check_poll(uss: Ultrasonic, is_auto: bool, exp_dist: float | None = None):
    # Save readings for comparison afterwards
    all_readings = []
    for i in range(1, ATTEMPTS + 1):
        sleep(2)
        if is_auto:
            # Test automatic polling
            print("\nAutomatic polling attempt ", i)
            sensor_readings = uss.poll()
        else:
            # Test ad-hoc polling
            print("\nAd-hoc polling attempt ", i)
            # send an ad-hoc polling request to the simulator.
            request_id = uss.send_ad_hoc_poll_request()
            print("Ad-hoc poll requests sent. Unique request Id number: ", request_id)
            sleep(3)
            # Ensure that the data has been processed before collecting.
            print(
                "Is ad-hoc request complete? ",
                uss.is_ad_hoc_poll_request_ready(request_id),
            )
            # Collect the data now that it has been computed.
            sensor_readings = uss.collect_ad_hoc_poll_request(request_id)
        print("Ultrasonic readings: ", sensor_readings)
        assert len(sensor_readings.values()) == 3 and len(sensor_readings.keys()) == 3
        for reading in sensor_readings.values():
            assert reading > 0
        if exp_dist:
            assert abs(sensor_readings["distance"] - exp_dist) <= 0.1
        all_readings.append(sensor_readings)

    for i in range(1, ATTEMPTS - 1):
        assert all_readings[0]["distance"] != all_readings[i]["distance"], "Readings don't get updated"
        assert all_readings[0]["windowMin"] != all_readings[i]["windowMin"], "Readings don't get updated"
        assert all_readings[0]["windowMax"] != all_readings[i]["windowMax"], "Readings don't get updated"

    assert abs(max(tmp["distance"] for tmp in all_readings) - min(tmp["distance"] for tmp in all_readings)) <= 0.1, "Readings inconsistent"
    assert abs(max(tmp["windowMax"] for tmp in all_readings) - min(tmp["windowMax"] for tmp in all_readings)) <= 0.1, "Readings inconsistent"
    assert abs(max(tmp["windowMin"] for tmp in all_readings) - min(tmp["windowMin"] for tmp in all_readings)) <= 0.1, "Readings inconsistent"

def test_ultrasonic(beamng: BeamNGpy):
    with beamng as bng:
        # Create a vehicle.
        vehicle = Vehicle("ego_vehicle", model="etki", licence="PYTHON", color="Red")
        obstacle = Vehicle("obstacle_vehicle", model="etki", licence="PYTHON", color="Blue")
        # Create a scenario.
        scenario = Scenario(
            "tech_ground",
            "ultrasonic_test",
            description="Testing the ultrasonic sensor",
        )
        # Add the vehicle to the scenario.
        scenario.add_vehicle(vehicle)
        scenario.add_vehicle(obstacle, pos=(0, -5, 0))
        scenario.make(bng)
        # Set simulator to 60hz temporal resolution
        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        print("Ultrasonic test start.")

        # Create a default ultrasonic sensor.
        ultrasonic1 = Ultrasonic("ultrasonic1", bng, vehicle, pos=(0, -2.3, 0.6))

        # Test the automatic polling functionality of the ultrasonic sensor, to make sure we retrieve the readings.
        sleep(2)
        check_poll(ultrasonic1, True, 0.406)

        # Test the ad-hoc polling functionality of the ultrasonic sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
        sleep(1)
        check_poll(ultrasonic1, False, 0.406)
        ultrasonic1.remove()
        print("Ultrasonic sensor removed.")

        # Create an ultrasonic sensor which has a negative requested update rate, and ensure that no readings are computed from it.
        ultrasonic2 = Ultrasonic(
            "ultrasonic2", bng, vehicle, requested_update_time=-1.0, pos=(0, -2.3, 0.6)
        )
        print("Testing an ultrasonic sensor with a negative requested update time...")
        sleep(2)
        sensor_readings = ultrasonic2.poll()
        print("Ultrasonic readings (should be zeros): ", sensor_readings)
        for reading in sensor_readings.values():
            assert reading >= 9999
        ultrasonic2.remove()

        # Recreate the first ultrasonic sensor.
        ultrasonic1 = Ultrasonic("ultrasonic1", bng, vehicle)

        # Test that the property getter function return the correct data which was set.
        sleep(1)
        print(
            "Property getter test.  The displayed values should be the values which were set during the creation of the ultrasonic sensors."
        )
        print("Sensor Name: ", ultrasonic1.name)
        print("Position: ", ultrasonic1.get_position())
        print("Direction: ", ultrasonic1.get_direction())
        print("Requested update time: ", ultrasonic1.get_requested_update_time())
        print("Priority: ", ultrasonic1.get_update_priority())
        print("Max Pending Requests: ", ultrasonic1.get_max_pending_requests())
        print("Is Visualised [Flag]: ", ultrasonic1.get_is_visualised())

        # Test that we can set the sensor core properties in the simulator from beamngpy.
        sleep(1)
        print(
            "Property setter test.  The displayed property values should be different from the previous values."
        )
        ultrasonic1.set_requested_update_time(0.3)
        print(
            "Newly-set Requested Update Time: ", ultrasonic1.get_requested_update_time()
        )
        ultrasonic1.set_update_priority(0.5)
        print("Newly-set Priority: ", ultrasonic1.get_update_priority())
        ultrasonic1.set_max_pending_requests(5)
        print(
            "Newly-set Max Pending Requests: ", ultrasonic1.get_max_pending_requests()
        )

        # Test changing the visibility of the sensor.
        print(
            "Test visibility mode.  Ultrasonic visibility should cycle between on and off 3 times, staying at each for 1 second."
        )
        sleep(1)
        ultrasonic1.set_is_visualised(False)
        sleep(1)
        ultrasonic1.set_is_visualised(True)
        sleep(1)
        ultrasonic1.set_is_visualised(False)
        sleep(1)
        ultrasonic1.set_is_visualised(True)
        sleep(1)
        ultrasonic1.set_is_visualised(False)
        sleep(1)
        ultrasonic1.set_is_visualised(True)

        ultrasonic1.remove()

        sleep(3)
        print("Ultrasonic test complete.")
        bng.ui.show_hud()


# Executing this file will perform various tests on all available functionality relating to the ultrasonic sensor.
# It is provided to give examples on how to use all ultrasonic sensor functions currently available in beamngpy.
if __name__ == "__main__":
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    test_ultrasonic(bng)
