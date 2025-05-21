from __future__ import annotations

from time import sleep
import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Radar

# Radar parameters
RANGE_MIN = 0.1
RANGE_MAX = 100.0
RESOLUTION = (200, 200)
FOV = 70

def check_consistency_and_update(radar: Radar, is_auto: bool):
    all_readings = []
    attempts = 3

    # Make sure we retrieve actively updated and consistent readings.
    for i in range(1, attempts + 1):
        sleep(2)
        if is_auto:
            print("\nAutomatic polling attempt ", i)
            sensor_readings = radar.poll()
        else:
            print("Ad-hoc poll request attempt ", i)
            # send an ad-hoc polling request to the simulator.
            request_id = radar.send_ad_hoc_poll_request()
            print("Ad-hoc poll requests sent. Unique request Id number: ", request_id)
            sleep(3)
            # Ensure that the data has been processed before collecting.
            print(
                "Is ad-hoc request complete? ",
                radar.is_ad_hoc_poll_request_ready(request_id),
            )
            # Collect the data now that it has been computed.
            sensor_readings = radar.collect_ad_hoc_poll_request(request_id)

        print("RADAR readings: ", sensor_readings[0:10])
        assert len(sensor_readings) > 0, "Readings not present"
        # Plot the data.
        # radar.plot_data(
        #     sensor_readings, RESOLUTION, FOV, RANGE_MIN, RANGE_MAX, 200, 200
        # )
        print("PASS: Readings present")
        all_readings.append(sensor_readings)

    # readings will often not match in length
    # if they have only a small difference, clip the longer to match the shorter
    for i in range(1, 3):
        if np.abs(all_readings[i].shape[0] - all_readings[0].shape[0]) < all_readings[0].shape[0] * 0.05:
            if all_readings[i].shape[0] - all_readings[0].shape[0] > 0:
                all_readings[i] = all_readings[i][:all_readings[0].shape[0], :]
            else:
                all_readings[0] = all_readings[0][:all_readings[i].shape[0], :]
                if i == 2:
                    all_readings[1] = all_readings[1][:all_readings[i].shape[0], :]
        else:
            raise AssertionError("Radar readings inconsistent")

    print("PASS: Readings consistent")

    assert np.any((all_readings[0] != all_readings[1]) | (all_readings[0] != all_readings[2])), "Readings don't get updated"
    print("PASS: Readings get updated")

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
        radar1 = Radar(
            "radar1",
            bng,
            vehicle,
            requested_update_time=0.01,
            pos=(0, 0, 1.7),
            dir=(0, -1, 0),
            up=(0, 0, 1),
            size=RESOLUTION,
            field_of_view_y=FOV,
            near_far_planes=(RANGE_MIN, RANGE_MAX),
            range_roundness=-2.0,
            range_cutoff_sensitivity=0.0,
            range_shape=0.23,
            range_focus=0.12,
            range_min_cutoff=0.5,
            range_direct_max_cutoff=RANGE_MAX,
        )

        sleep(10)

        # Test the automatic polling functionality of the RADAR sensor.
        check_consistency_and_update(radar1, False)

        # Test the ad-hoc polling functionality of the RADAR sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
        check_consistency_and_update(radar1, True)

        radar1.remove()
        print("RADAR sensor removed.")

        # Create a RADAR sensor which has a negative requested update rate, and ensure that no readings are computed from it.
        radar2 = Radar("radar2", bng, vehicle, requested_update_time=-1.0)
        print("Testing an ultrasonic sensor with a negative requested update time...")
        sleep(2)
        sensor_readings = radar2.poll()
        print("RADAR readings (should be zeros): ", sensor_readings)
        assert sensor_readings.shape[0] == 0
        radar2.remove()

        # Recreate the first RADAR sensor, with default parameters.
        radar1 = Radar("radar1", bng, vehicle)

        # Test that the property getter function return the correct data which was set.
        sleep(1)
        print(
            "Property getter test.  The displayed values should be the values which were set during the creation of the ultrasonic sensors."
        )
        print("Sensor Name: ", radar1.name)
        assert radar1.name == "radar1"
        print("Position: ", radar1.get_position())
        print("Direction: ", radar1.get_direction())
        print("Requested update time: ", radar1.get_requested_update_time())
        assert np.round(radar1.get_requested_update_time(), 3) == 0.1
        print("Priority: ", radar1.get_update_priority())
        assert np.round(radar1.get_update_priority(), 3) == 0
        print("Max Pending Requests: ", radar1.get_max_pending_requests())
        assert radar1.get_max_pending_requests() == 10

        # Test that we can set the sensor core properties in the simulator from beamngpy.
        sleep(1)
        print(
            "Property setter test.  The displayed property values should be different from the previous values."
        )
        radar1.set_requested_update_time(0.3)
        print("Newly-set Requested Update Time: ", radar1.get_requested_update_time())
        assert np.round(radar1.get_requested_update_time(), 3) == 0.3
        radar1.set_update_priority(0.5)
        print("Newly-set Priority: ", radar1.get_update_priority())
        assert np.round(radar1.get_update_priority(), 3) == 0.5
        radar1.set_max_pending_requests(5)
        print("Newly-set Max Pending Requests: ", radar1.get_max_pending_requests())
        assert radar1.get_max_pending_requests() == 5

        radar1.remove()

        sleep(3)
        print("RADAR test complete.")
        bng.ui.show_hud()


# Executing this file will perform various tests on all available functionality relating to the RADAR sensor.
# It is provided to give examples on how to use all RADAR sensor functions currently available in beamngpy.
if __name__ == "__main__":
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    test_radar(bng)
