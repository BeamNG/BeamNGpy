from __future__ import annotations

from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Lidar


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

        # Test the automatic polling functionality of the LiDAR sensor, to make sure we retrieve the point cloud data via shared memory.
        sleep(2)
        data = lidar1.poll()
        print("LiDAR point cloud data (automatic polling): ", data["pointCloud"])
        print("LiDAR colour data (automatic polling): ", data["colours"])

        # Test the ad-hoc polling functionality of the LiDAR sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
        sleep(1)
        print("Ad-hoc poll request test.")
        # send an ad-hoc polling request to the simulator.
        request_id = lidar1.send_ad_hoc_poll_request()
        print("Ad-hoc poll requests sent. Unique request Id number: ", request_id)
        sleep(3)
        # Ensure that the data has been processed before collecting.
        print(
            "Is ad-hoc request complete? ",
            lidar1.is_ad_hoc_poll_request_ready(request_id),
        )
        # Collect the data now that it has been computed.
        data_ad_hoc = lidar1.collect_ad_hoc_poll_request(request_id)
        print("LiDAR point cloud data (ad-hoc polling): ", data_ad_hoc["pointCloud"])
        print("LiDAR colour data (ad-hoc polling): ", data_ad_hoc["colours"])
        lidar1.remove()
        print("LiDAR sensor removed.")

        # Now create a default LiDAR sensor which does not use shared memory, and perform the same tests.
        lidar2 = Lidar("lidar2", bng, vehicle, is_using_shared_memory=False)
        print("Testing a LiDAR sensor which DOES NOT use shared memory...")

        # Test the automatic polling functionality of this LiDAR sensor, to make sure we retrieve the point cloud data via the lua socket.
        sleep(2)
        data = lidar2.poll()
        print("LiDAR point cloud data (automatic polling): ", data["pointCloud"])
        print("LiDAR colour data (automatic polling): ", data["colours"])

        # Test the ad-hoc polling functionality of the LiDAR sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
        sleep(1)
        print("Ad-hoc poll request test.")
        # send an ad-hoc polling request to the simulator.
        request_id = lidar2.send_ad_hoc_poll_request()
        print("Ad-hoc poll requests sent. Unique request Id number: ", request_id)
        sleep(3)
        # Ensure that the data has been processed before collecting.
        print(
            "Is ad-hoc request 1 complete? ",
            lidar2.is_ad_hoc_poll_request_ready(request_id),
        )
        # Collect the data now that it has been computed.
        data_ad_hoc = lidar2.collect_ad_hoc_poll_request(request_id)
        print("LiDAR point cloud data (ad-hoc polling): ", data_ad_hoc["pointCloud"])
        print("LiDAR colour data (ad-hoc polling): ", data_ad_hoc["colours"])
        lidar2.remove()
        print("LiDAR sensor removed.")

        # Create a LiDAR sensor which has a negative requested update rate, and ensure that no readings are computed from it.
        lidar3 = Lidar("lidar3", bng, vehicle, requested_update_time=-1.0)
        print(
            "Testing a LiDAR sensor with a negative requested update time (WITH shared memory)..."
        )
        sleep(2)
        data = lidar3.poll()
        print("LiDAR point cloud data (should be zeros): ", data["pointCloud"])
        print("LiDAR colour data (should be zeros): ", data["colours"])
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
        sleep(2)
        data = lidar4.poll()
        print("LiDAR point cloud data (should be empty): ", data["pointCloud"])
        print("LiDAR colour data (should be empty): ", data["colours"])
        lidar4.remove()

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
        sleep(1)
        lidar1.set_is_visualised(False)
        sleep(1)
        lidar1.set_is_visualised(True)
        sleep(1)
        lidar1.set_is_visualised(False)
        sleep(1)
        lidar1.set_is_visualised(True)
        sleep(1)
        lidar1.set_is_visualised(False)
        sleep(1)
        lidar1.set_is_visualised(True)

        # Test switching between depth mode and annotation mode.
        print(
            "Test LiDAR mode.  LiDAR should cycle between DEPTH MODE and ANNOTATION MODE, staying at each for 1 second."
        )
        sleep(1)
        lidar1.set_is_annotated(False)
        sleep(1)
        lidar1.set_is_annotated(True)
        sleep(1)
        lidar1.set_is_annotated(False)
        sleep(1)
        lidar1.set_is_annotated(True)
        sleep(1)
        lidar1.set_is_annotated(False)
        sleep(1)
        lidar1.set_is_annotated(True)

        lidar1.remove()

        sleep(3)
        print("LiDAR test complete.")


# Executing this file will perform various tests on all available functionality relating to the LiDAR sensor.
# It is provided to give examples on how to use all LiDAR sensor functions currently available in beamngpy.
if __name__ == "__main__":
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    test_lidar(bng)
