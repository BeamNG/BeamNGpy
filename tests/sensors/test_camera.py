from __future__ import annotations

import time
import matplotlib.pyplot as plt
import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera


def test_camera(beamng: BeamNGpy):
    with beamng as bng:
        # Set up the vehicle and scenario
        vehicle = Vehicle("ego_vehicle", model="etki", license="PYTHON", color="Green")
        scenario = Scenario("tech_ground", "camera_test", description="Testing the camera sensor")
        scenario.add_vehicle(vehicle)
        scenario.make(bng)

        # Load and start the scenario
        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        # Allow the scenario to stabilize
        time.sleep(5)

        # Create camera sensors
        cameras = [
            Camera("camera1", bng, vehicle, is_using_shared_memory=True, pos=(-5, 0, 1), dir=(1, 0, 0),
                   field_of_view_y=70, is_render_annotations=True, is_render_depth=True, near_far_planes=(0.1, 1000),
                   resolution=(512, 512)),
            Camera("camera2", bng, vehicle, is_using_shared_memory=False, pos=(5, 0, 1), dir=(-1, 0, 0),
                   field_of_view_y=70, is_render_annotations=True, is_render_depth=True, near_far_planes=(0.1, 1000),
                   resolution=(512, 512)),
            Camera("camera3", bng, vehicle, is_using_shared_memory=False, pos=(0, 5, 1), dir=(0, -1, 0), up=(1, 0, 1),
                   field_of_view_y=70, is_render_annotations=True, is_render_depth=True, near_far_planes=(0.1, 1000),
                   resolution=(512, 512))
        ]

        # Poll and display images from each camera
        for cam in cameras:
            time.sleep(2)  # Allow the camera sensor to initialize
            print(f"Polling images from {cam.name}...")
            images = cam.poll()

            if images:
                if "colour" in images and images["colour"]:
                    plt.imshow(np.asarray(images["colour"].convert("RGB")))
                    plt.title(f"{cam.name} - Colour Image")
                    plt.show()
                else:
                    print(f"{cam.name}: No colour data available.")

                if "annotation" in images and images["annotation"]:
                    plt.imshow(np.asarray(images["annotation"].convert("RGB")))
                    plt.title(f"{cam.name} - Annotation Image")
                    plt.show()
                else:
                    print(f"{cam.name}: No annotation data available.")

                if "depth" in images and images["depth"]:
                    plt.imshow(np.asarray(images["depth"].convert("RGB")))
                    plt.title(f"{cam.name} - Depth Image")
                    plt.show()
                else:
                    print(f"{cam.name}: No depth data available.")

        # Remove all cameras from the simulation
        for cam in cameras:
            cam.remove()

        time.sleep(3)
        print("Camera test complete.")

#
# Executing this file will perform various tests on all available functionality relating to the camera sensor.
# It is provided to give examples on how to use all camera sensor functions currently available in beamngpy.
if __name__ == "__main__":
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    test_camera(bng)
