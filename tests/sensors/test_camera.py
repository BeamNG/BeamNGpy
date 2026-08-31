from __future__ import annotations

from contextlib import contextmanager
from time import sleep
from typing import Iterator

import matplotlib.pyplot as plt
import numpy as np

from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import Camera
from tests.sensors.helpers import (
    loaded_scenario,
    make_module_scene_fixture,
    poll_auto_or_adhoc,
    using_sensor,
)

TOTAL_FIELDS = 4
VISUALISE = False
ATTEMPTS = 1 if VISUALISE else 3

def check_field(sensor_readings, field: str, cam: Camera):
    assert field in sensor_readings.keys()
    assert (
        sensor_readings[field].shape[0] * sensor_readings[field].shape[1]
        == cam.resolution[0] * cam.resolution[1]
    )
    assert (sensor_readings[field] != 0).any()
    assert (sensor_readings[field] >= 0).all() and (sensor_readings[field] <= 255).all()

def check_poll(
    cam: Camera,
    is_auto: bool,
    exp_semantic: int | None = None,
    exp_instance: int | None = None,
):
    fields = []
    if cam.is_render_colours:
        fields.append("colour")
    if cam.is_render_annotations:
        fields.append("annotation")
    if cam.is_render_instance:
        fields.append("instance")
    if cam.is_render_depth:
        fields.append("depth")

    last_readings = {}
    for i in range(1, ATTEMPTS + 1):
        sleep(2)
        if len(fields) == TOTAL_FIELDS:
            print("\nFull poll attempt ", i)
            sensor_readings = cam.get_full_poll_request()
        else:
            sensor_readings = poll_auto_or_adhoc(cam, is_auto, attempt=i)

        for field in fields:
            sensor_readings[field] = np.asarray(sensor_readings[field].convert("RGB"))
            if VISUALISE:
                plt.imshow(sensor_readings[field])
                plt.show()
            else:
                print(field + " readings: \n")
                print(sensor_readings[field])
            check_field(sensor_readings, field, cam)
            last_readings[field] = sensor_readings[field]

        if exp_semantic != None:
            assert (
                len(np.unique(sensor_readings["annotation"].reshape(-1, 3), axis=0))
                == exp_semantic
            ), "Incorrect number of solid colors"
        if exp_instance != None:
            assert (
                len(np.unique(sensor_readings["instance"].reshape(-1, 3), axis=0))
                == exp_instance
            ), "Incorrect number of solid colors"

    return last_readings

def check_stream(cam: Camera):
    """Validate stream/stream_raw shared-memory reads.

    PollCamera with shared memory refreshes the SHM buffers that stream reads from.
    """
    assert cam.is_streaming and cam.is_using_shared_memory

    colour = None
    for i in range(1, ATTEMPTS + 1):
        sleep(2)
        cam.poll_raw()
        raw = cam.stream_raw()
        colour = raw.get("colour")
        nonzero = bool(colour) and any(bytearray(colour))
        print(
            f"Stream attempt {i}: bytes={0 if not colour else len(colour)} nonzero={nonzero}"
        )
        if nonzero:
            break

    assert colour is not None
    assert len(colour) == cam.resolution[0] * cam.resolution[1] * 4
    assert any(bytearray(colour)), "shared-memory colour empty after PollCamera + stream_raw"

    streamed = cam.stream()
    colour_img = np.asarray(streamed["colour"].convert("RGB"))
    check_field({"colour": colour_img}, "colour", cam)

@contextmanager
def camera_scenario(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    vehicle = Vehicle("ego_vehicle", model="etki", license="PYTHON", color="Green")
    with loaded_scenario(
        beamng,
        level="smallgrid",
        name="camera_test",
        description="Testing the camera sensor",
        vehicles=[(vehicle, {})],
        label="Camera test",
    ) as scene:
        yield scene

camera_scene = make_module_scene_fixture(camera_scenario)

def test_camera_shmem(camera_scene):
    """Shared memory, to the left of the vehicle, facing towards it."""
    bng, vehicle = camera_scene
    with using_sensor(
        Camera(
            "camera1", bng, vehicle,
            is_using_shared_memory=True,
            pos=(-5, 0, 1), dir=(1, 0, 0), field_of_view_y=70,
            is_render_annotations=True, is_render_depth=True, is_render_instance=True,
            near_far_planes=(0.1, 100), resolution=(512, 512),
        ),
    ) as cam:
        sleep(5)
        print("Testing camera 1...")
        check_poll(cam, True, 3, 3)

        sleep(1)
        print(
            "Property getter test.  The displayed values should be the values which were set during the creation of the camera sensors."
        )
        print("Camera Name: ", cam.name)
        print("Position: ", cam.get_position())
        print("Direction: ", cam.get_direction())
        print("Requested update time: ", cam.get_requested_update_time())
        print("Priority: ", cam.get_update_priority())
        print("Max Pending Requests: ", cam.get_max_pending_requests())

        sleep(1)
        print(
            "Property setter test.  The displayed property values should be different from the previous values."
        )
        cam.set_requested_update_time(0.3)
        print("Newly-set Requested Update Time: ", cam.get_requested_update_time())
        cam.set_update_priority(0.5)
        print("Newly-set Priority: ", cam.get_update_priority())
        cam.set_max_pending_requests(5)
        print("Newly-set Max Pending Requests: ", cam.get_max_pending_requests())

        print(
            "out of range pixel. should be [-1, -1]: ",
            cam.world_point_to_pixel((1e7, 1e7, 1e7)),
        )
        print(
            "pixel at vehicle. should be around center: ",
            cam.world_point_to_pixel((0, 0, 0)),
        )
        print("behind pixel1: ", cam.world_point_to_pixel((-5.1, 0, 1)))
        print(
            "off-center pixel. should be near bottom-right corner: ",
            cam.world_point_to_pixel((0, -2.7, -2)),
        )
        print(
            "off-center pixel. should be near top-left corner near [0, 0]: ",
            cam.world_point_to_pixel((0, 3, 4)),
        )

def test_camera_no_shmem(camera_scene):
    """No shared memory, to the left of the vehicle, facing towards it."""
    bng, vehicle = camera_scene
    with using_sensor(
        Camera(
            "camera2", bng, vehicle,
            is_using_shared_memory=False,
            pos=(-5, 0, 1), dir=(1, 0, 0), field_of_view_y=70,
            is_render_annotations=True, is_render_depth=True, is_render_instance=True,
            near_far_planes=(0.1, 100), resolution=(512, 512),
        ),
    ) as cam:
        sleep(5)
        print("Testing camera 2...")
        check_poll(cam, True, 3, 3)

def test_camera_translucent_depth(camera_scene):
    """Depth differs when is_render_translucent_as_opaque_depth is enabled."""
    bng, vehicle = camera_scene
    cam_cfg = dict(
        pos=(-5, 0, 1),
        dir=(1, 0, 0),
        field_of_view_y=70,
        is_render_annotations=True,
        is_render_depth=True,
        is_render_instance=True,
        near_far_planes=(0.1, 100),
        resolution=(512, 512),
    )
    with using_sensor(
        Camera("camera1", bng, vehicle, is_using_shared_memory=True, **cam_cfg),
    ) as cam1:
        sleep(5)
        cam1_readings = check_poll(cam1, True, 3, 3)
    with using_sensor(
        Camera(
            "camera2",
            bng,
            vehicle,
            is_using_shared_memory=False,
            is_render_translucent_as_opaque_depth=True,
            **cam_cfg,
        ),
    ) as cam2:
        sleep(5)
        cam2_readings = check_poll(cam2, True, 3, 3)

    print("Testing is_render_translucent_as_opaque_depth...")
    assert not np.array_equal(
        cam1_readings["depth"][250, 260], cam2_readings["depth"][250, 260]
    ), (
        f"Depth at pixel ({260}, {250}) should differ when "
        "is_render_translucent_as_opaque_depth is True vs False"
    )

def test_camera_shmem_ad_hoc(camera_scene):
    """Shared memory, to the right of the vehicle, automatic and ad-hoc polling."""
    bng, vehicle = camera_scene
    with using_sensor(
        Camera(
            "camera3", bng, vehicle,
            is_using_shared_memory=True,
            pos=(5, 0, 1), dir=(-1, 0, 0), field_of_view_y=70,
            is_render_annotations=True, is_render_depth=True,
            near_far_planes=(0.1, 100), resolution=(512, 512),
        ),
    ) as cam:
        sleep(5)
        print("Testing camera 3...")
        check_poll(cam, True, 3)
        check_poll(cam, False, 3)

def test_camera_shmem_stream(camera_scene):
    """Shared-memory streaming: poll fills SHM, then stream/stream_raw read it back."""
    bng, vehicle = camera_scene
    with using_sensor(
        Camera(
            "camera_stream", bng, vehicle,
            requested_update_time=0.05,
            is_using_shared_memory=True, is_streaming=True,
            pos=(-5, 0, 1), dir=(1, 0, 0), field_of_view_y=70,
            is_render_annotations=True, is_render_depth=True,
            near_far_planes=(0.1, 100), resolution=(512, 512),
        ),
    ) as cam:
        sleep(5)
        print("Testing camera stream...")
        check_stream(cam)

def test_camera_no_shmem_oblique(camera_scene):
    """No shared memory, oblique angle to the world, then repositioned."""
    bng, vehicle = camera_scene
    with using_sensor(
        Camera(
            "camera4", bng, vehicle,
            is_using_shared_memory=False,
            pos=(0, 5, 1), dir=(0, -1, 0), up=(1, 0, 1), field_of_view_y=70,
            is_render_annotations=True, is_render_depth=True,
            near_far_planes=(0.1, 100), resolution=(512, 512),
        ),
    ) as cam:
        sleep(5)
        print("Testing camera 4...")
        check_poll(cam, True, 3)
        check_poll(cam, False, 3)

        cam.set_position((-10, 0, 1))
        print("Newly-set Position: ", cam.get_position())
        cam.set_direction((-1, 0, 0))
        print("Newly-set Direction: ", cam.get_direction())

        print("Camera 4 images (after altering position, direction, and up vectors)...")
        sleep(1)
        images = cam.poll()
        if VISUALISE:
            for field in ["colour", "annotation", "depth"]:
                plt.imshow(np.asarray(images[field].convert("RGB")))
                plt.show()

def test_camera_idle(camera_scene):
    """A negative requested update time must not automatically poll."""
    bng, vehicle = camera_scene
    print(
        "Negative update time test.  The next 3 images should be blank, since the camera is set to not poll."
    )
    with using_sensor(
        Camera(
            "idle cam", bng, vehicle,
            requested_update_time=-1.0, is_using_shared_memory=True,
            pos=(-5, 0, 1), dir=(1, 0, 0),
            is_render_annotations=True, is_render_depth=True,
            field_of_view_y=70, near_far_planes=(0.1, 1000), resolution=(512, 512),
        ),
    ) as idle_cam:
        sleep(3)
        images = idle_cam.poll()
        for field in ["colour", "annotation", "depth"]:
            images[field] = np.asarray(images[field].convert("RGB"))
            if VISUALISE:
                plt.imshow(images[field])
                plt.show()
            assert np.all(images[field] == 0)
