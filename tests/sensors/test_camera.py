from __future__ import annotations

from contextlib import contextmanager
from time import sleep
from typing import Iterator

import matplotlib.pyplot as plt
import numpy as np
import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera

TOTAL_FIELDS = 4
VISUALISE = False
ATTEMPTS = 1 if VISUALISE else 3

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
_STANDARD_POLL_FIELDS = ("colour", "annotation", "depth")

TESTS = (
    "test_camera_shmem",
    "test_camera_no_shmem",
    "test_camera_translucent_depth",
    "test_camera_shmem_ad_hoc",
    "test_camera_shmem_stream",
    "test_camera_no_shmem_oblique",
    "test_camera_idle",
)


def make_camera(bng, vehicle, name: str, **overrides) -> Camera:
    return Camera(name, bng, vehicle, **{**_BASE_CAM, **overrides})


@contextmanager
def managed_camera(bng, vehicle, name: str, **overrides) -> Iterator[Camera]:
    cam = make_camera(bng, vehicle, name, **overrides)
    try:
        yield cam
    finally:
        cam.remove()


def enabled_fields(cam: Camera, *, include_instance: bool = True) -> list[str]:
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


def to_rgb(image) -> np.ndarray:
    return np.asarray(image.convert("RGB"))


def maybe_show(field: str, arr: np.ndarray) -> None:
    if VISUALISE:
        plt.imshow(arr)
        plt.show()
    else:
        print(f"{field} readings: \n{arr}")


def check_field(sensor_readings, field: str, cam: Camera):
    assert field in sensor_readings
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
    fields = enabled_fields(cam)
    all_readings = {field: [] for field in fields}

    for i in range(1, ATTEMPTS + 1):
        sleep(2)
        if len(fields) == TOTAL_FIELDS:
            print("\nFull poll attempt ", i)
            sensor_readings = cam.get_full_poll_request()
        elif is_auto:
            print("\nAutomatic polling attempt ", i)
            sensor_readings = cam.poll()
        else:
            print("\nAd-hoc polling attempt ", i)
            request_id = cam.send_ad_hoc_poll_request()
            sleep(3)
            print(
                "Is ad-hoc request 1 complete? ",
                cam.is_ad_hoc_poll_request_ready(request_id),
            )
            sensor_readings = cam.collect_ad_hoc_poll_request(request_id)

        for field in fields:
            sensor_readings[field] = to_rgb(sensor_readings[field])
            maybe_show(field, sensor_readings[field])
            check_field(sensor_readings, field, cam)
            all_readings[field].append(sensor_readings[field])

        for field, expected in (("annotation", exp_semantic), ("instance", exp_instance)):
            if expected is not None:
                unique = len(np.unique(sensor_readings[field].reshape(-1, 3), axis=0))
                assert unique == expected, "Incorrect number of solid colors"

    return all_readings


def check_stream(cam: Camera):
    """Validate stream/stream_raw shared-memory reads.

    PollCamera with shared memory refreshes the SHM buffers that stream reads from.
    """
    assert cam.is_streaming and cam.is_using_shared_memory

    fields = enabled_fields(cam, include_instance=False)
    expected_bytes = _RESOLUTION[0] * _RESOLUTION[1] * 4
    raw = None
    for i in range(1, ATTEMPTS + 1):
        sleep(2)
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

    streamed_raw = cam.stream()
    streamed = {field: to_rgb(streamed_raw[field]) for field in fields}
    for field in fields:
        check_field(streamed, field, cam)


@contextmanager
def camera_scenario(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    """Same scenario as the original combined test, loaded once per split test."""
    with beamng as bng:
        vehicle = Vehicle("ego_vehicle", model="etki", license="PYTHON", color="Green")
        scenario = Scenario(
            "smallgrid", "camera_test", description="Testing the camera sensor"
        )
        scenario.add_vehicle(vehicle)
        scenario.make(bng)
        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        print("Camera test start.")
        yield bng, vehicle

        sleep(3)
        print("Camera test complete.")
        bng.ui.show_hud()


@pytest.fixture
def camera_scene(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    with camera_scenario(beamng) as scene:
        yield scene


def test_camera_shmem(camera_scene):
    """Camera 1: shared memory, to the left of the vehicle, facing towards it."""
    bng, vehicle = camera_scene
    with managed_camera(
        bng, vehicle, "camera1", is_using_shared_memory=True, **_LEFT_VIEW, **_FULL_RENDER
    ) as cam1:
        sleep(5)
        print("Testing camera 1...")
        check_poll(cam1, True, 3, 3)

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

        print(
            "out of range pixel. should be [-1, -1]: ",
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


def test_camera_no_shmem(camera_scene):
    """Camera 2: no shared memory, to the left of the vehicle, facing towards it."""
    bng, vehicle = camera_scene
    with managed_camera(
        bng, vehicle, "camera2", is_using_shared_memory=False, **_LEFT_VIEW, **_FULL_RENDER
    ) as cam2:
        sleep(5)
        print("Testing camera 2...")
        check_poll(cam2, True, 3, 3)


def test_camera_translucent_depth(camera_scene):
    """Same view as camera1/camera2; translucent-as-opaque depth must change depth output."""
    bng, vehicle = camera_scene
    with managed_camera(
        bng, vehicle, "camera1_translucent", is_using_shared_memory=True, **_LEFT_VIEW, **_FULL_RENDER
    ) as cam1:
        with managed_camera(
            bng,
            vehicle,
            "camera2_translucent",
            is_using_shared_memory=False,
            is_render_translucent_as_opaque_depth=True,
            **_LEFT_VIEW,
            **_FULL_RENDER,
        ) as cam2:
            sleep(5)
            print("Testing is_render_translucent_as_opaque_depth...")
            cam1_readings = check_poll(cam1, True, 3, 3)
            cam2_readings = check_poll(cam2, True, 3, 3)
            assert not np.array_equal(
                cam1_readings["depth"][-1][250, 260],
                cam2_readings["depth"][-1][250, 260],
            ), (
                f"Depth at pixel ({260}, {250}) should differ when "
                "is_render_translucent_as_opaque_depth is True vs False"
            )


def test_camera_shmem_ad_hoc(camera_scene):
    """Camera 3: shared memory, to the right of the vehicle, automatic and ad-hoc polling."""
    bng, vehicle = camera_scene
    with managed_camera(
        bng,
        vehicle,
        "camera3",
        is_using_shared_memory=True,
        pos=(5, 0, 1),
        dir=(-1, 0, 0),
        is_render_annotations=True,
        is_render_depth=True,
    ) as cam3:
        sleep(5)
        print("Testing camera 3...")
        check_poll(cam3, True, 3)
        check_poll(cam3, False, 3)


def test_camera_shmem_stream(camera_scene):
    """Shared-memory streaming: poll fills SHM, then stream/stream_raw read it back."""
    bng, vehicle = camera_scene
    with managed_camera(
        bng,
        vehicle,
        "camera_stream",
        requested_update_time=0.05,
        is_using_shared_memory=True,
        is_streaming=True,
        is_render_annotations=True,
        is_render_depth=True,
        **_LEFT_VIEW,
    ) as cam:
        sleep(5)
        print("Testing camera stream...")
        check_stream(cam)


def test_camera_no_shmem_oblique(camera_scene):
    """Camera 4: no shared memory, oblique angle to the world, then repositioned."""
    bng, vehicle = camera_scene
    with managed_camera(
        bng,
        vehicle,
        "camera4",
        is_using_shared_memory=False,
        pos=(0, 5, 1),
        dir=(0, -1, 0),
        up=(1, 0, 1),
        is_render_annotations=True,
        is_render_depth=True,
    ) as cam4:
        sleep(5)
        print("Testing camera 4...")
        check_poll(cam4, True, 3)
        check_poll(cam4, False, 3)

        cam4.set_position((-10, 0, 1))
        print("Newly-set Position: ", cam4.get_position())
        cam4.set_direction((-1, 0, 0))
        print("Newly-set Direction: ", cam4.get_direction())

        print("Camera 4 images (after altering position, direction, and up vectors)...")
        sleep(1)
        images = cam4.poll()
        for field in enabled_fields(cam4, include_instance=False):
            maybe_show(field, to_rgb(images[field]))


def test_camera_idle(camera_scene):
    """Idle camera: a negative requested update time must not automatically poll."""
    bng, vehicle = camera_scene
    print(
        "Negative update time test.  The next 3 images should be blank, since the camera is set to not poll."
    )
    with managed_camera(
        bng,
        vehicle,
        "idle cam",
        requested_update_time=-1.0,
        is_using_shared_memory=True,
        near_far_planes=(0.1, 1000),
        is_render_annotations=True,
        is_render_depth=True,
        **_LEFT_VIEW,
    ) as idle_cam:
        sleep(3)
        images = idle_cam.poll()
        for field in _STANDARD_POLL_FIELDS:
            images[field] = to_rgb(images[field])
            maybe_show(field, images[field])
            assert np.all(images[field] == 0)


if __name__ == "__main__":
    set_up_simple_logging()

    bng = BeamNGpy("localhost", 25252, quit_on_close=False)
    test_funcs = {name: globals()[name] for name in TESTS}
    for name in TESTS:
        print("Running ", name)
        with camera_scenario(bng) as scene:
            test_funcs[name](scene)
