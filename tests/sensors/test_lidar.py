from __future__ import annotations

from time import sleep

import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Lidar

ATTEMPTS = 3

def validate_distribution(pointCloud):
    x, y = pointCloud[:, 0], pointCloud[:, 1]
    phi = np.degrees(np.arctan2(y, x))

    # Calculate number of points in every region (top-down view, divided into 16 regions)
    counts, bin_edges = np.histogram(phi, bins=16, range=(-180, 180))

    min_count = np.min(counts)
    max_count = np.max(counts)
    count_difference = max_count - min_count

    return count_difference < max_count / 5

def polling_check(lidar: Lidar, is_auto: bool, is_360: bool, is_rotate: bool):
    data_total = []
    prev_first_point = None
    data = None

    # Test the polling functionality of the LiDAR sensor, to make sure we retrieve the point cloud data.
    # Additionally checks are made for data presence, distribution (360 only) and consistency
    for i in range(0, ATTEMPTS):
        sleep(2)
        if is_auto:
            print("\nAutomatic polling attempt ", i + 1)
            data = lidar.poll()
        else:
            print("\nAd-hoc poll request test attempt ", i + 1)
            # send an ad-hoc polling request to the simulator.
            request_id = lidar.send_ad_hoc_poll_request()
            print("Ad-hoc poll requests sent. Unique request Id number: ", request_id)
            sleep(3)
            # Ensure that the data has been processed before collecting.
            print(
                "Is ad-hoc request complete? ",
                lidar.is_ad_hoc_poll_request_ready(request_id),
            )
            # Collect the data now that it has been computed.
            data = lidar.collect_ad_hoc_poll_request(request_id)

        print("LiDAR point cloud data: ", data["pointCloud"])
        print("LiDAR colour data: ", data["colours"])

        assert len(data["pointCloud"]) > 0
        print("PASS: Point cloud is present")

        if is_360:
            assert validate_distribution(data["pointCloud"]), "360 point cloud data is not uniform."
            print("PASS: Data is uniform")

        if i == 0: prev_first_point = data["pointCloud"][0]
        if not is_rotate: data_total.append(len(data["pointCloud"]))
        assert len(data["colours"]) > 0
        print("PASS: Colour data is present")

    assert prev_first_point is not None and (data["pointCloud"][0] != prev_first_point).all(), "Point cloud doesn't get updated"
    print("PASS: Point cloud gets updated")

    if not is_rotate:
        assert np.max(data_total) - np.min(data_total) < np.max(data_total) * 0.01, "360 or static point cloud data is inconsistent"
    print("PASS: LiDAR point cloud data is consistent")

def negative_update_test(lidar: Lidar):
    sleep(2)
    data = lidar.poll()
    print("LiDAR point cloud data (should be zeros): ", data["pointCloud"])
    print("LiDAR colour data (should be zeros): ", data["colours"])

    assert len(data["pointCloud"]) == 0 and len(data["colours"]) == 0
    print("PASS: No data is present")

def test_lidar(beamng: BeamNGpy):
    with beamng as bng:
        vehicle = Vehicle(
            "ego_vehicle", model="etki", license="PYTHON", color="Blue"
        )  # Create a vehicle.
        scenario = Scenario(
            "tech_ground", "lidar_test", description="Testing the LiDAR sensor"
        )  # Create a scenario.
        # Add the vehicle to the scenario.
        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(bng)
        # Set simulator to 60 Hz temporal resolution
        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        print("LiDAR test start.")

        # Create a default LiDAR sensor which uses shared memory.
        lidar1 = Lidar("lidar1", bng, vehicle, is_using_shared_memory=True)
        print("Testing a LiDAR sensor which uses shared memory...")

        # Test the polling functionality of the LiDAR sensor, to make sure we retrieve the point cloud data via shared memory.
        polling_check(lidar1, True, True, False)
        polling_check(lidar1, False, True, False)
        lidar1.remove()
        print("LiDAR sensor removed.")

        # Now create a default LiDAR sensor which does not use shared memory, and perform the same tests.
        lidar2 = Lidar("lidar2", bng, vehicle, is_using_shared_memory=False)
        print("Testing a LiDAR sensor which DOES NOT use shared memory...")

        # Test the polling functionality of the LiDAR sensor, to make sure we retrieve the point cloud data without shared memory.
        polling_check(lidar2, True, True, False)
        polling_check(lidar2, False, True, False)
        lidar2.remove()
        print("LiDAR sensor removed.")

        # Create a LiDAR sensor which has a negative requested update rate, and ensure that no readings are computed from it.
        lidar3 = Lidar("lidar3", bng, vehicle, requested_update_time=-1.0)
        print(
            "Testing a LiDAR sensor with a negative requested update time (WITH shared memory)..."
        )
        negative_update_test(lidar3)
        lidar3.remove()

        # And the same again WITHOUT shared memory.
        lidar4 = Lidar(
            "lidar3",
            bng,
            vehicle,
            requested_update_time=-1.0,
            is_using_shared_memory=False,
        )
        print(
            "Testing a LiDAR sensor with a negative requested update time (WITHOUT shared memory)..."
        )
        negative_update_test(lidar4)
        lidar4.remove()

        # Create a LiDAR sensor that works in LFO mode
        lidar5 = Lidar("lidar5", bng, vehicle, is_using_shared_memory=True, is_360_mode=False, is_rotate_mode=True, frequency=1, horizontal_angle=90)
        print("Testing a LiDAR sensor in LFO mode...")

        # Test the polling functionality of the LiDAR sensor in LFO mode.
        polling_check(lidar5, True, False, True)
        polling_check(lidar5, False, False, True)
        lidar5.remove()
        print("LiDAR sensor removed.")

        # Create a LiDAR sensor that works in fixed mode
        lidar6 = Lidar("lidar6", bng, vehicle, is_using_shared_memory=True, is_360_mode=False, horizontal_angle=90)
        print("Testing a LiDAR sensor in static mode...")

        # Test the polling functionality of the LiDAR sensor in LFO mode.
        polling_check(lidar6, True, False, False)
        polling_check(lidar6, False, False, False)
        lidar6.remove()
        print("LiDAR sensor removed.")

        # Recreate the first LiDAR sensor.
        lidar1 = Lidar("lidar1", bng, vehicle)

        # Test that the property getter function return the correct data which was set.
        sleep(1)
        print(
            "Property getter test.  The displayed values should be the values which were set during the creation of the LiDAR sensors."
        )
        print("Sensor Name: ", lidar1.name)
        print("Position: ", lidar1.get_position())
        print("Direction: ", lidar1.get_direction())
        print("Requested update time: ", lidar1.get_requested_update_time())
        print("Priority: ", lidar1.get_update_priority())
        print("Max Pending Requests: ", lidar1.get_max_pending_requests())
        print("Is Visualised [Flag]: ", lidar1.get_is_visualised())
        print("Is Annotated [Flag]: ", lidar1.get_is_annotated())

        # Test that we can set the sensor core properties in the simulator from beamngpy.
        sleep(1)
        print(
            "Property setter test.  The displayed property values should be different from the previous values."
        )
        lidar1.set_requested_update_time(0.3)
        print("Newly-set Requested Update Time: ", lidar1.get_requested_update_time())
        lidar1.set_update_priority(0.5)
        print("Newly-set Priority: ", lidar1.get_update_priority())
        lidar1.set_max_pending_requests(5)
        print("Newly-set Max Pending Requests: ", lidar1.get_max_pending_requests())

        # Test changing the visibility of the sensor.
        print(
            "Test visibility mode.  LiDAR visibility should cycle between on and off 3 times, staying at each for 1 second."
        )
        for i in range(0, 6):
            sleep(1)
            lidar1.set_is_visualised(not (i % 2 == 0))

        # Test switching between depth mode and annotation mode.
        print(
            "Test LiDAR mode.  LiDAR should cycle between DEPTH MODE and ANNOTATION MODE, staying at each for 1 second."
        )
        for i in range(0, 6):
            sleep(1)
            lidar1.set_is_annotated(not (i % 2 == 0))

        lidar1.remove()

        sleep(3)
        print("LiDAR test complete.")
        bng.ui.show_hud()

# Executing this file will perform various tests on all available functionality relating to the LiDAR sensor.
# It is provided to give examples on how to use all LiDAR sensor functions currently available in beamngpy.
if __name__ == "__main__":
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    test_lidar(bng)
