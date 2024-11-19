from __future__ import annotations

from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Radar


def test_radar(beamng: BeamNGpy):
    with beamng as bng:
        # Create a vehicle.
        vehicle = Vehicle("ego_vehicle", model="etki", licence="PYTHON", color="Red")
        # Create a scenario.
        scenario = Scenario(
            "italy", "radar_test", description="Testing the RADAR sensor"
        )
        # Add the vehicle to the scenario.
        scenario.add_vehicle(
            vehicle,
            pos=(237.90, -894.42, 246.10),
            rot_quat=(0.0173, -0.0019, -0.6354, 0.7720),
        )
        scenario.make(bng)
        # Set simulator to 60hz temporal resolution
        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        print("RADAR test start.")

        # Create a RADAR sensor.
        RANGE_MIN = 0.1
        RANGE_MAX = 100.0
        RESOLUTION = (200, 200)
        FOV = 70
        radar1 = Radar(
            "radar1",
            bng,
            vehicle,
            requested_update_time=0.01,
            pos=(0, 0, 1.7),
            dir=(0, -1, 0),
            up=(0, 0, 1),
            resolution=RESOLUTION,
            field_of_view_y=FOV,
            near_far_planes=(RANGE_MIN, RANGE_MAX),
            range_roundness=-2.0,
            range_cutoff_sensitivity=0.0,
            range_shape=0.23,
            range_focus=0.12,
            range_min_cutoff=0.5,
            range_direct_max_cutoff=RANGE_MAX,
        )

        # Test the automatic polling functionality of the RADAR sensor, to make sure we retrieve the readings.
        sleep(2)
        sensor_readings = radar1.poll()
        print("RADAR readings (automatic polling): ", sensor_readings[0:10])
        # Plot the data.
        radar1.plot_data(
            sensor_readings, RESOLUTION, FOV, RANGE_MIN, RANGE_MAX, 200, 200
        )

        # Test the ad-hoc polling functionality of the RADAR sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
        sleep(1)
        print("Ad-hoc poll request test.")
        # send an ad-hoc polling request to the simulator.
        request_id = radar1.send_ad_hoc_poll_request()
        print("Ad-hoc poll requests sent. Unique request Id number: ", request_id)
        sleep(3)
        # Ensure that the data has been processed before collecting.
        print(
            "Is ad-hoc request complete? ",
            radar1.is_ad_hoc_poll_request_ready(request_id),
        )
        # Collect the data now that it has been computed.
        sensor_readings_ad_hoc = radar1.collect_ad_hoc_poll_request(request_id)
        print("RADAR readings (ad-hoc polling): ", sensor_readings_ad_hoc[0:10])
        radar1.remove()
        print("RADAR sensor removed.")

        # Create a RADAR sensor which has a negative requested update rate, and ensure that no readings are computed from it.
        radar2 = Radar("radar2", bng, vehicle, requested_update_time=-1.0)
        print("Testing an ultrasonic sensor with a negative requested update time...")
        sleep(2)
        sensor_readings = radar2.poll()
        print("RADAR readings (should be zeros): ", sensor_readings)
        radar2.remove()

        # Recreate the first RADAR sensor, with default parameters.
        radar1 = Radar("radar1", bng, vehicle)

        # Test that the property getter function return the correct data which was set.
        sleep(1)
        print(
            "Property getter test.  The displayed values should be the values which were set during the creation of the ultrasonic sensors."
        )
        print("Sensor Name: ", radar1.name)
        print("Position: ", radar1.get_position())
        print("Direction: ", radar1.get_direction())
        print("Requested update time: ", radar1.get_requested_update_time())
        print("Priority: ", radar1.get_update_priority())
        print("Max Pending Requests: ", radar1.get_max_pending_requests())

        # Test that we can set the sensor core properties in the simulator from beamngpy.
        sleep(1)
        print(
            "Property setter test.  The displayed property values should be different from the previous values."
        )
        radar1.set_requested_update_time(0.3)
        print("Newly-set Requested Update Time: ", radar1.get_requested_update_time())
        radar1.set_update_priority(0.5)
        print("Newly-set Priority: ", radar1.get_update_priority())
        radar1.set_max_pending_requests(5)
        print("Newly-set Max Pending Requests: ", radar1.get_max_pending_requests())

        radar1.remove()

        sleep(3)
        print("RADAR test complete.")


# Executing this file will perform various tests on all available functionality relating to the RADAR sensor.
# It is provided to give examples on how to use all RADAR sensor functions currently available in beamngpy.
if __name__ == "__main__":
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    test_radar(bng)
