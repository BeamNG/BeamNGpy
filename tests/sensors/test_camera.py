from __future__ import annotations

from time import sleep

import numpy as np

from beamngpy.sensors import Camera

import tests.sensors.helpers as h

_RESOLUTION = (512, 512)
_BASE_CAM = dict(
    field_of_view_y=70,
    near_far_planes=(0.1, 100),
    resolution=_RESOLUTION,
)
_LEFT_VIEW = dict(pos=(-5, 0, 1), dir=(1, 0, 0))
_FULL_RENDER = dict(
    is_render_annotations=True,
    is_render_depth=True,
    is_render_instance=True,
)
_RENDER_STD = dict(is_render_annotations=True, is_render_depth=True)
_LEFT_FULL = {**_LEFT_VIEW, **_FULL_RENDER}


def make_camera(bng, vehicle, name: str, **overrides) -> Camera:
    return Camera(name, bng, vehicle, **{**_BASE_CAM, **overrides})


def camera_poll_fields(cam: Camera, *, include_instance: bool = True) -> list[str]:
    fields = []
    if cam.is_render_colours:
        fields.append("colour")
    if cam.is_render_annotations:
        fields.append("annotation")
    if include_instance and cam.is_render_instance:
        fields.append("instance")
    if cam.is_render_depth:
        fields.append("depth")
    return fields


def check_field(arr: np.ndarray, cam: Camera):
    assert (
        arr.shape[0] * arr.shape[1]
        == cam.resolution[0] * cam.resolution[1]
    )
    assert (arr != 0).any()
    assert (arr >= 0).all() and (arr <= 255).all()


def check_poll(
    cam: Camera, test_name: str, is_auto: bool, exp_semantic: int | None = None, exp_instance: int | None = None,
):
    fields = camera_poll_fields(cam)
    last_readings = {}

    for i in range(1, h.ATTEMPTS + 1):
        sleep(1)
        if "instance" in fields:
            raw = cam.get_full_poll_request()
        else:
            raw = h.poll_auto_or_adhoc(cam, is_auto, attempt=i)
        sensor_readings = {f: np.asarray(raw[f].convert("RGB")) for f in fields}
        for field in fields:
            if not h.VISUALISE:
                print(field + " readings: \n")
                print(sensor_readings[field])
            check_field(sensor_readings[field], cam)
        last_readings = {f: sensor_readings[f] for f in fields}
        h.save_sensor_images("camera", test_name, sensor_readings, tag=f"attempt{i}")

        for field, expected in (("annotation", exp_semantic), ("instance", exp_instance)):
            if expected is not None:
                unique = len(np.unique(sensor_readings[field].reshape(-1, 3), axis=0))
                assert unique == expected, "Incorrect number of solid colors"

    return last_readings


def check_stream(cam: Camera):
    """Validate stream/stream_raw shared-memory reads.

    PollCamera with shared memory refreshes the SHM buffers that stream reads from.
    """
    assert cam.is_streaming and cam.is_using_shared_memory

    fields = camera_poll_fields(cam, include_instance=False)
    expected_bytes = cam.resolution[0] * cam.resolution[1] * 4
    raw = None
    for i in range(1, h.ATTEMPTS + 1):
        sleep(1)
        cam.poll_raw()
        raw = cam.stream_raw()
        all_nonzero = all(
            bool(raw.get(field)) and any(bytearray(raw[field])) for field in fields
        )
        print(
            f"Stream attempt {i}: "
            + ", ".join(
                f"{field}={0 if not raw.get(field) else len(raw[field])}"
                for field in fields
            )
            + f" all_nonzero={all_nonzero}"
        )
        if all_nonzero:
            break

    assert raw is not None
    for field in fields:
        field_bytes = raw[field]
        assert len(field_bytes) == expected_bytes
        assert any(bytearray(field_bytes)), (
            f"shared-memory {field} empty after PollCamera + stream_raw"
        )

    streamed = {k: np.asarray(v.convert("RGB")) for k, v in cam.stream().items()}
    for field in fields:
        check_field(streamed[field], cam)


camera_scene = h.module_scene(
    "camera_test", "Testing the camera sensor", "Camera test", color="Green",
)


def test_camera_shmem(camera_scene, request):
    """Shared memory, to the left of the vehicle, facing towards it."""
    bng, vehicle = camera_scene
    with h.using_sensor(
        make_camera(bng, vehicle, "camera1", is_using_shared_memory=True, **_LEFT_FULL),
    ) as cam:
        print("Testing camera 1...")
        check_poll(cam, request.node.name, True, 3, 3)

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


def test_camera_property_roundtrips(camera_scene):
    """Assert getter/setter round-trips for camera pose, scheduling, frustum, and exposure."""
    bng, vehicle = camera_scene
    with h.using_sensor(
        make_camera(bng, vehicle, "camera_props", is_using_shared_memory=False, **_LEFT_FULL),
    ) as cam:
        h.exercise_roundtrip(
            cam,
            h.scheduling_pose_cases(
                cam, (-4.0, 0.5, 1.2), direction=(-0.9, 0.1, 0.0),
                extra=[
                    ("get_resolution", (256, 256), cam.set_resolution),
                    ("get_field_of_view_x", 80.0, cam.set_field_of_view_x),
                    ("get_near_far_planes", (0.2, 50.0), cam.set_near_far_planes),
                ],
            ),
        )
        assert cam.get_is_auto_exposure() is True
        cam.set_manual_exposure(0.002)
        h.assert_float_close(cam.get_manual_exposure(), 0.002)
        assert cam.get_is_auto_exposure() is False
        cam.set_auto_exposure(True)
        assert cam.get_is_auto_exposure() is True


def test_camera_no_shmem(camera_scene, request):
    """No shared memory, to the left of the vehicle, facing towards it."""
    bng, vehicle = camera_scene
    with h.using_sensor(
        make_camera(bng, vehicle, "camera2", is_using_shared_memory=False, **_LEFT_FULL),
    ) as cam:
        print("Testing camera 2...")
        check_poll(cam, request.node.name, True, 3, 3)


def test_camera_translucent_depth(camera_scene, request):
    """Depth differs when is_render_translucent_as_opaque_depth is enabled."""
    bng, vehicle = camera_scene
    with h.using_sensor(
        make_camera(bng, vehicle, "camera1", is_using_shared_memory=True, **_LEFT_FULL),
    ) as cam1:
        cam1_readings = check_poll(cam1, request.node.name, True, 3, 3)
    with h.using_sensor(
        make_camera(
            bng,
            vehicle,
            "camera2",
            is_using_shared_memory=False,
            is_render_translucent_as_opaque_depth=True,
            **_LEFT_FULL,
        ),
    ) as cam2:
        cam2_readings = check_poll(cam2, request.node.name, True, 3, 3)

    print("Testing is_render_translucent_as_opaque_depth...")
    assert not np.array_equal(
        cam1_readings["depth"][250, 260], cam2_readings["depth"][250, 260]
    ), (
        f"Depth at pixel ({260}, {250}) should differ when "
        "is_render_translucent_as_opaque_depth is True vs False"
    )


def test_camera_shmem_ad_hoc(camera_scene, request):
    """Shared memory, to the right of the vehicle, automatic and ad-hoc polling."""
    bng, vehicle = camera_scene
    with h.using_sensor(
        make_camera(
            bng, vehicle, "camera3", is_using_shared_memory=True,
            pos=(5, 0, 1), dir=(-1, 0, 0), **_RENDER_STD,
        ),
    ) as cam:
        print("Testing camera 3...")
        check_poll(cam, request.node.name, True, 3)
        check_poll(cam, request.node.name, False, 3)


def test_camera_shmem_stream(camera_scene):
    """Shared-memory streaming: poll fills SHM, then stream/stream_raw read it back."""
    bng, vehicle = camera_scene
    with h.using_sensor(
        make_camera(
            bng, vehicle, "camera_stream",
            requested_update_time=0.05,
            is_using_shared_memory=True, is_streaming=True,
            **_LEFT_VIEW, **_RENDER_STD,
        ),
    ) as cam:
        print("Testing camera stream...")
        check_stream(cam)


def test_camera_no_shmem_oblique(camera_scene, request):
    """No shared memory, oblique angle to the world, then repositioned."""
    bng, vehicle = camera_scene
    with h.using_sensor(
        make_camera(
            bng, vehicle, "camera4", is_using_shared_memory=False,
            pos=(0, 5, 1), dir=(0, -1, 0), up=(1, 0, 1), **_RENDER_STD,
        ),
    ) as cam:
        print("Testing camera 4...")
        check_poll(cam, request.node.name, True, 3)
        check_poll(cam, request.node.name, False, 3)

        cam.set_position((-10, 0, 1))
        print("Newly-set Position: ", cam.get_position())
        cam.set_direction((-1, 0, 0))
        print("Newly-set Direction: ", cam.get_direction())

        print("Camera 4 images (after altering position, direction, and up vectors)...")
        sleep(1)
        repositioned = {k: np.asarray(v.convert("RGB")) for k, v in cam.poll().items()}
        h.save_sensor_images("camera", request.node.name, repositioned, tag="repositioned")


def test_camera_idle(camera_scene, request):
    """A negative requested update time must not automatically poll."""
    bng, vehicle = camera_scene
    print(
        "Negative update time test.  The next 3 images should be blank, since the camera is set to not poll."
    )
    with h.using_sensor(
        make_camera(
            bng, vehicle, "idle cam",
            requested_update_time=-1.0, is_using_shared_memory=True,
            near_far_planes=(0.1, 1000), **_LEFT_VIEW, **_RENDER_STD,
        ),
    ) as idle_cam:
        sleep(1)
        images = {k: np.asarray(v.convert("RGB")) for k, v in idle_cam.poll().items()}
        h.save_sensor_images("camera", request.node.name, images)
        print("Sensor readings (should be empty): ", images)
        assert all(np.all(images[f] == 0) for f in images)
