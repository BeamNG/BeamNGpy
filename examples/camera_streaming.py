from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera

if __name__ == "__main__":
    beamng = BeamNGpy("localhost", 25252)
    beamng.open()

    scenario = Scenario("italy", "camera_streaming")

    ego = Vehicle("ego", model="etk800", color="White")
    scenario.add_vehicle(
        ego, pos=(237.90, -894.42, 246.10), rot_quat=(0.0173, -0.0019, -0.6354, 0.7720)
    )

    scenario.make(beamng)

    beamng.settings.set_deterministic(60)

    beamng.control.pause()
    beamng.scenario.load(scenario)
    beamng.scenario.start()

    ego.ai.set_mode("traffic")

    camera = Camera(
        "camera1",
        beamng,
        ego,
        requested_update_time=0.01,
        is_using_shared_memory=True,
        pos=(-0.3, 1, 2),
        dir=(0, -1, 0),
        field_of_view_y=70,
        near_far_planes=(0.1, 1000),
        resolution=(1024, 1024),
        is_streaming=True,
        is_render_annotations=True,
        is_render_instance=True,
        is_render_depth=True,
    )

    for i in range(10):
        beamng.control.step(10)

        # Getting raw bytes from the simulator -> the fastest you can get
        raw_readings = camera.stream_raw()
        # if you don't want to use streaming, then use this instead
        # raw_readings = camera.poll_raw()

        if i % 100 == 0:
            print("Showing images from the stream...")

            # Getting PIL.Image images created from the raw bytes -> slower
            images = camera.stream()
            # if you don't want to use streaming, then use this instead
            # raw_readings = camera.poll()

            if camera.is_render_colours:
                images["colour"].show()
            if camera.is_render_annotations:
                images["annotation"].show()
            if camera.is_render_depth:
                images["depth"].show()
    beamng.control.resume()
    input("Press Enter to exit...")
