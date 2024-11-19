from __future__ import annotations

from time import sleep

import matplotlib.pyplot as plt
import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera


def test_camera(beamng: BeamNGpy):
    with beamng as bng:
        vehicle = Vehicle(
            "ego_vehicle", model="etki", license="PYTHON", color="Green"
        )  # Create a vehicle.
        scenario = Scenario(
            "tech_ground", "camera_test", description="Testing the camera sensor"
        )  # Create a scenario.
        # Add the vehicle to the scenario.
        scenario.add_vehicle(vehicle)
        scenario.make(bng)
        # Set simulator to 60hz temporal resolution
        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        # Create some camera sensors in the simulation.
        print("Camera test start.")

        # Create a camera sensor which uses shared memory. This is placed to the left of the vehicle, facing towards the vehicle.
        cam1 = Camera(
            "camera1",
            bng,
            vehicle,
            is_using_shared_memory=True,
            pos=(-5, 0, 1),
            dir=(1, 0, 0),
            field_of_view_y=70,
            is_render_annotations=True,
            is_render_depth=True,
            near_far_planes=(0.1, 1000),
            resolution=(512, 512),
        )

        # Create a camera sensor which does not use shared memory (data will be send back across the socket). This is placed to the right of the vehicle,
        # facing towards the vehicle.
        cam2 = Camera(
            "camera2",
            bng,
            vehicle,
            is_using_shared_memory=False,
            pos=(5, 0, 1),
            dir=(-1, 0, 0),
            field_of_view_y=70,
            is_render_annotations=True,
            is_render_depth=True,
            near_far_planes=(0.1, 1000),
            resolution=(512, 512),
        )

        # Create a camera sensor which has an oblique angle to the world
        cam3 = Camera(
            "camera3",
            bng,
            vehicle,
            is_using_shared_memory=False,
            pos=(0, 5, 1),
            dir=(0, -1, 0),
            up=(1, 0, 1),
            field_of_view_y=70,
            is_render_annotations=True,
            is_render_depth=True,
            near_far_planes=(0.1, 1000),
            resolution=(512, 512),
        )

        # Test the image data by polling the camera sensors.
        # We use each camera sensor to take: i) colour, ii) annotation, and iii) depth images, from their given positions.
        sleep(2)
        print("Camera 1 images...")
        images = cam1.poll()
        plt.imshow(np.asarray(images["colour"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images["annotation"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images["depth"].convert("RGB")))
        plt.show()

        print("Camera 2 images...")
        images = cam2.poll()
        plt.imshow(np.asarray(images["colour"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images["annotation"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images["depth"].convert("RGB")))
        plt.show()

        print("Camera 3 images...")
        images = cam3.poll()
        plt.imshow(np.asarray(images["colour"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images["annotation"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images["depth"].convert("RGB")))
        plt.show()

        # Test that the property getter function return the correct data which was set.
        sleep(1)
        print(
            "Property getter test.  The displayed values should be the values which were set during the creation of the camera sensors."
        )
        print("Camera Name: ", cam1.name)
        print("Position: ", cam1.get_position())
        print("Direction: ", cam1.get_direction())
        print("Requested update time: ", cam1.get_requested_update_time())
        print("Priority: ", cam1.get_update_priority())
        print("Max Pending Requests: ", cam1.get_max_pending_requests())

        # Test that we can set the sensor core properties in the simulator from beamngpy.
        sleep(1)
        print(
            "Property setter test.  The displayed property values should be different from the previous values."
        )
        cam1.set_requested_update_time(0.3)
        print("Newly-set Requested Update Time: ", cam1.get_requested_update_time())
        cam1.set_update_priority(0.5)
        print("Newly-set Priority: ", cam1.get_update_priority())
        cam1.set_max_pending_requests(5)
        print("Newly-set Max Pending Requests: ", cam1.get_max_pending_requests())
        cam3.set_position((-10, 0, 1))
        print("Newly-set Position: ", cam3.get_position())
        cam3.set_direction((-1, 0, 0))
        print("Newly-set Direction: ", cam3.get_direction())

        print("Camera 3 images (after altering position, direction, and up vectors)...")
        sleep(1)
        images = cam3.poll()
        plt.imshow(np.asarray(images["colour"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images["annotation"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images["depth"].convert("RGB")))
        plt.show()

        # Test the world-space to camera pixel functionality.
        print(
            "out of range pixel. should be [-1, -1: ",
            cam1.world_point_to_pixel((1e7, 1e7, 1e7)),
        )
        print(
            "pixel at vehicle. should be around center: ",
            cam1.world_point_to_pixel((0, 0, 0)),
        )
        print("behind pixel1: ", cam1.world_point_to_pixel((-5.1, 0, 1)))
        print(
            "off-center pixel. should be near bottom-right corner: ",
            cam1.world_point_to_pixel((0, -2.7, -2)),
        )
        print(
            "off-center pixel. should be near top-left corner near [0, 0]: ",
            cam1.world_point_to_pixel((0, 3, 4)),
        )

        # Test the ad-hoc polling functionality of the camera sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
        sleep(1)
        print(
            "Ad-hoc poll request test.  The next 6 images come from ad-hoc requests sent to 2 camera sensors. They should contain scene data as before."
        )
        # send a request on the shared memory sensor (data should come back over the socket, either way).
        request_id_1 = cam1.send_ad_hoc_poll_request()
        request_id_2 = (
            cam2.send_ad_hoc_poll_request()
        )  # send a request on the non shared memory sensor.
        print(
            "Ad-hoc poll requests sent. Unique request Id numbers: ",
            request_id_1,
            request_id_2,
        )
        sleep(3)
        print(
            "Is ad-hoc request 1 complete? ",
            cam1.is_ad_hoc_poll_request_ready(request_id_1),
        )
        print(
            "Is ad-hoc request 2 complete? ",
            cam2.is_ad_hoc_poll_request_ready(request_id_2),
        )

        images1 = cam1.collect_ad_hoc_poll_request(
            request_id_1
        )  # Display the image data from request 1.
        plt.imshow(np.asarray(images1["colour"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images1["annotation"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images1["depth"].convert("RGB")))
        plt.show()

        images2 = cam1.collect_ad_hoc_poll_request(
            request_id_2
        )  # Display the image data from request 2.
        plt.imshow(np.asarray(images2["colour"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images2["annotation"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images2["depth"].convert("RGB")))
        plt.show()

        # Test that a camera sensor with a negative requested update time performs as it should (it should not automatically poll for readings).
        # We create a camera with a negative update time, then attempt to poll it. The images here should not be an image of the scene.
        print(
            "Negative update time test.  The next 3 images should be blank, since the camera is set to not poll."
        )
        idle_cam = Camera(
            "idle cam",
            bng,
            vehicle,
            requested_update_time=-1.0,
            is_using_shared_memory=True,
            pos=(-5, 0, 1),
            dir=(1, 0, 0),
            is_render_annotations=True,
            is_render_depth=True,
            field_of_view_y=70,
            near_far_planes=(0.1, 1000),
            resolution=(512, 512),
        )
        sleep(3)
        images = idle_cam.poll()
        plt.imshow(np.asarray(images["colour"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images["annotation"].convert("RGB")))
        plt.show()
        plt.imshow(np.asarray(images["depth"].convert("RGB")))
        plt.show()

        # Remove all the camera sensors from the simulation.
        cam1.remove()
        cam2.remove()
        cam3.remove()
        idle_cam.remove()

        sleep(3)
        print("Camera test complete.")


# Executing this file will perform various tests on all available functionality relating to the camera sensor.
# It is provided to give examples on how to use all camera sensor functions currently available in beamngpy.
if __name__ == "__main__":
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    test_camera(bng)
