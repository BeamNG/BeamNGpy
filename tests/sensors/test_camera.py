from __future__ import annotations

from time import sleep

import matplotlib.pyplot as plt
import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera

ATTEMPTS = 3
TOTAL_FIELDS = 4
VISUALISE = False

def check_field(sensor_readings, field: str, cam: Camera):
    # Check field is present, correct size, non-zero and with valid rgb values
    assert field in sensor_readings.keys()
    assert sensor_readings[field].shape[0] * sensor_readings[field].shape[1] == cam.resolution[0] * cam.resolution[1]
    assert (sensor_readings[field] != 0).any()
    assert (sensor_readings[field] >= 0).all() and (sensor_readings[field] <= 255).all()

def check_poll(cam: Camera, is_auto: bool, visualize_img: bool, exp_semantic: int | None = None, exp_instance: int | None = None):
    # Generate fields
    fields = []
    if cam.is_render_colours: fields.append("colour")
    if cam.is_render_annotations: fields.append("annotation")
    if cam.is_render_instance: fields.append("instance")
    if cam.is_render_depth: fields.append("depth")

    # Save readings for comparison afterwards
    all_readings = {}
    for field in fields:
        all_readings[field] = []

    # Test the image data by polling the camera sensors.
    # i) colour, ii) semantic annotation, iii) instance annotation, and iv) depth images, from their given positions.
    for i in range(1, ATTEMPTS + 1):
        sleep(2)
        if len(fields) == TOTAL_FIELDS:
            print("\nFull poll attempt ", i)
            sensor_readings = cam.get_full_poll_request()
        elif is_auto:
            print("\nAutomatic polling attempt ", i)
            sensor_readings = cam.poll()
        else:
            # Test ad-hoc polling
            print("\nAd-hoc polling attempt ", i)
            request_id = cam.send_ad_hoc_poll_request()
            sleep(3)
            print(
                "Is ad-hoc request 1 complete? ",
                cam.is_ad_hoc_poll_request_ready(request_id),
            )
            sensor_readings = cam.collect_ad_hoc_poll_request(request_id)

        for field in fields:
            sensor_readings[field] = np.asarray(sensor_readings[field].convert("RGB"))
            print(field + " readings: \n")
            print(sensor_readings[field])
            if visualize_img:
                plt.imshow(sensor_readings[field])
                plt.show()
            check_field(sensor_readings, field, cam)
            all_readings[field].append(sensor_readings[field])

        # Check semantic and instance annotations have correct number of solid colors
        if exp_semantic != None:
            assert len(np.unique(sensor_readings["annotation"].reshape(-1, 3), axis=0)) == exp_semantic, "Incorrect number of solid colors"
        if exp_instance != None:
            assert len(np.unique(sensor_readings["instance"].reshape(-1, 3), axis=0)) == exp_instance, "Incorrect number of solid colors"

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
            is_render_instance=True,
            near_far_planes=(0.1, 100),
            resolution=(512, 512),
        )

        # Create a camera sensor which doesn't support full poll. This is placed to the left of the vehicle, facing towards the vehicle.
        cam2 = Camera(
            "camera2",
            bng,
            vehicle,
            is_using_shared_memory=False,
            pos=(-5, 0, 1),
            dir=(1, 0, 0),
            field_of_view_y=70,
            is_render_annotations=True,
            is_render_depth=True,
            is_render_instance=True,
            near_far_planes=(0.1, 100),
            resolution=(512, 512),
        )

        # Create a camera sensor which does not use shared memory (data will be send back across the socket). This is placed to the right of the vehicle,
        # facing towards the vehicle.
        cam3 = Camera(
            "camera3",
            bng,
            vehicle,
            is_using_shared_memory=True,
            pos=(5, 0, 1),
            dir=(-1, 0, 0),
            field_of_view_y=70,
            is_render_annotations=True,
            is_render_depth=True,
            near_far_planes=(0.1, 100),
            resolution=(512, 512),
        )

        # Create a camera sensor which has an oblique angle to the world
        cam4 = Camera(
            "camera4",
            bng,
            vehicle,
            is_using_shared_memory=False,
            pos=(0, 5, 1),
            dir=(0, -1, 0),
            up=(1, 0, 1),
            field_of_view_y=70,
            is_render_annotations=True,
            is_render_depth=True,
            near_far_planes=(0.1, 100),
            resolution=(512, 512),
        )

        # Test the image data by polling the camera sensors.
        # We use each camera sensor to take: i) colour, ii) annotation, and iii) depth images, from their given positions.
        sleep(5)
        print("Testing camera 1...")
        check_poll(cam1, True, VISUALISE, 3, 3)

        print("Testing camera 2...")
        check_poll(cam2, True, VISUALISE, 3, 3)

        print("Testing camera 3...")
        check_poll(cam3, True, VISUALISE, 3)
        check_poll(cam3, False, VISUALISE, 3)

        print("Testing camera 4...")
        check_poll(cam4, True, VISUALISE, 3)
        check_poll(cam4, False, VISUALISE, 3)

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
        cam4.set_position((-10, 0, 1))
        print("Newly-set Position: ", cam4.get_position())
        cam4.set_direction((-1, 0, 0))
        print("Newly-set Direction: ", cam4.get_direction())

        print("Camera 4 images (after altering position, direction, and up vectors)...")
        sleep(1)
        images = cam4.poll()
        if VISUALISE:
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
        for field in ["colour", "annotation", "depth"]:
            images[field] = np.asarray(images[field].convert("RGB"))
            if VISUALISE:
                plt.imshow(images[field])
                plt.show()
            assert np.all(images[field] == 0)

        # Remove all the camera sensors from the simulation.
        cam1.remove()
        cam2.remove()
        cam3.remove()
        cam4.remove()
        idle_cam.remove()

        sleep(3)
        print("Camera test complete.")
        bng.ui.show_hud()


# Executing this file will perform various tests on all available functionality relating to the camera sensor.
# It is provided to give examples on how to use all camera sensor functions currently available in beamngpy.
if __name__ == "__main__":
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    test_camera(bng)
