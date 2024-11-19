from __future__ import annotations

from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Ultrasonic


def test_ultrasonic(beamng: BeamNGpy):
    with beamng as bng:
        # Create a vehicle.
        vehicle = Vehicle("ego_vehicle", model="etki", licence="PYTHON", color="Red")
        # Create a scenario.
        scenario = Scenario(
            "tech_ground",
            "ultrasonic_test",
            description="Testing the ultrasonic sensor",
        )
        # Add the vehicle to the scenario.
        scenario.add_vehicle(vehicle)
        scenario.make(bng)
        # Set simulator to 60hz temporal resolution
        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        print("Ultrasonic test start.")

        # Create a default ultrasonic sensor.
        ultrasonic1 = Ultrasonic("ultrasonic1", bng, vehicle)

        # Test the automatic polling functionality of the ultrasonic sensor, to make sure we retrieve the readings.
        sleep(2)
        sensor_readings = ultrasonic1.poll()
        print("Ultrasonic readings (automatic polling): ", sensor_readings)

        # Test the ad-hoc polling functionality of the ultrasonic sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
        sleep(1)
        print("Ad-hoc poll request test.")
        # send an ad-hoc polling request to the simulator.
        request_id = ultrasonic1.send_ad_hoc_poll_request()
        print("Ad-hoc poll requests sent. Unique request Id number: ", request_id)
        sleep(3)
        # Ensure that the data has been processed before collecting.
        print(
            "Is ad-hoc request complete? ",
            ultrasonic1.is_ad_hoc_poll_request_ready(request_id),
        )
        # Collect the data now that it has been computed.
        sensor_readings_ad_hoc = ultrasonic1.collect_ad_hoc_poll_request(request_id)
        print("Ultrasonic readings (ad-hoc polling): ", sensor_readings_ad_hoc)
        ultrasonic1.remove()
        print("Ultrasonic sensor removed.")

        # Create an ultrasonic sensor which has a negative requested update rate, and ensure that no readings are computed from it.
        ultrasonic2 = Ultrasonic(
            "ultrasonic2", bng, vehicle, requested_update_time=-1.0
        )
        print("Testing an ultrasonic sensor with a negative requested update time...")
        sleep(2)
        sensor_readings = ultrasonic2.poll()
        print("Ultrasonic readings (should be zeros): ", sensor_readings)
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


# Executing this file will perform various tests on all available functionality relating to the ultrasonic sensor.
# It is provided to give examples on how to use all ultrasonic sensor functions currently available in beamngpy.
if __name__ == "__main__":
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    test_ultrasonic(bng)
