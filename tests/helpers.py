from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from math import sqrt
from pathlib import Path
from time import sleep
from typing import Any, Callable, Union

import numpy as np
import pytest
from PIL import Image

from beamngpy import BeamNGpy, Road, Scenario, Vehicle

VISUALISE = False
ATTEMPTS = 3
TEST_LEVEL = "smallgrid"
TEST_VEHICLE_MODEL = "etk800"
OTHER_VEHICLE_ID = "other_vehicle"
SETTLE_AFTER_TELEPORT_S = 2.0

# Obstacle ahead of ego (-Y). LiDAR needs >= ~9 m separation; ultrasonic max range is shorter.
OBSTACLE_START = (0.0, -10.0, 0.21)
LIDAR_OBSTACLE_STEP_Y = 0.5
USS_OBSTACLE_START = (0.0, -8.0, 0.21)
USS_OBSTACLE_STEP_Y = 0.25

SENSOR_TEST_DIR = (0, -1, 0)
SENSOR_TEST_UP = (0, 0, 1)

# A straight road collapses to two navgraph nodes named after the road; this is its far end.
TEST_ROAD_END_NODE = "adas_test_road2"

# Root directory for VISUALISE=True test outputs (PNGs, plots). One subfolder per sensor type.
_OUTPUT_ROOT = Path(__file__).resolve().parent / "sensors" / "output"


RoundtripCase = Union[
    tuple[str, Any, Callable[[Any], None]],
    tuple[str, Any, Callable[[Any], None], dict[str, Any]],
]


def scheduling_pose_cases(
    sensor,
    pos: tuple[float, float, float],
    direction: tuple[float, float, float] = SENSOR_TEST_DIR,
    up: tuple[float, float, float] = SENSOR_TEST_UP,
    extra: list[RoundtripCase] | None = None,
) -> list[RoundtripCase]:
    cases: list[RoundtripCase] = [
        ("get_requested_update_time", 0.25, sensor.set_requested_update_time),
        ("get_update_priority", 0.4, sensor.set_update_priority),
        ("get_max_pending_requests", 4, sensor.set_max_pending_requests),
        ("get_position", pos, sensor.set_position, {"world_space": False}),
        ("get_direction", direction, sensor.set_direction),
        ("get_up", up, sensor.set_up),
    ]
    if extra:
        cases.extend(extra)
    return cases


def assert_negative_update(sensor, is_empty, sleep_s: float = 1.0) -> None:
    sleep(sleep_s)
    readings = sensor.poll()
    print("Sensor readings (should be empty): ", readings)
    assert is_empty(readings)


def wait_until_poll_valid(
    bng: BeamNGpy, sensor, predicate, *, max_attempts: int = 5, step: int = 60,
) -> Any:
    readings = sensor.poll()
    for _ in range(max_attempts):
        if predicate(readings):
            return readings
        bng.control.step(step, wait=True)
        sleep(0.2)
        readings = sensor.poll()
    return readings


def assert_polls_change(values) -> None:
    ref = values[0]
    for v in values[1:]:
        if isinstance(ref, np.ndarray):
            if np.any(ref != v):
                return
        elif ref != v:
            return
    assert False, "Readings don't get updated"


def assert_duplicate_poll_consistent(sensor, is_auto: bool, count_fn) -> None:
    """Two polls at the same scene pose should agree within tolerance."""
    if is_auto:
        sleep(1)
        first = count_fn(sensor.poll())
        sleep(1)
        second = count_fn(sensor.poll())
    else:
        request_id = sensor.send_ad_hoc_poll_request()
        sleep(1)
        first = count_fn(sensor.collect_ad_hoc_poll_request(request_id))
        request_id = sensor.send_ad_hoc_poll_request()
        sleep(1)
        second = count_fn(sensor.collect_ad_hoc_poll_request(request_id))
    assert abs(first - second) / first < 0.01, (
        f"duplicate polls inconsistent at fixed pose: {first} vs {second}"
    )


def save_sensor_images(sensor_type: str, test_name: str, readings: dict[str, np.ndarray], tag: str | None = None) -> None:
    """Write {test_name}[_{tag}]_{field}.png under output/{sensor_type}/ when VISUALISE."""
    if not VISUALISE:
        return
    (_OUTPUT_ROOT / sensor_type).mkdir(parents=True, exist_ok=True)
    prefix = test_name if tag is None else f"{test_name}_{tag}"
    for field, arr in readings.items():
        Image.fromarray(arr).save(_OUTPUT_ROOT / sensor_type / f"{prefix}_{field}.png")


def save_radar_plot(
    radar, test_name: str, attempt: int, readings,
    resolution: tuple[int, int], fov: float, near_far: tuple[float, float],
) -> None:
    if not VISUALISE:
        return
    out_path = _OUTPUT_ROOT / "radar" / f"{test_name}_attempt{attempt}.png"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    radar.save_plot(str(out_path), readings, resolution, fov, near_far[0], near_far[1], *resolution)


@contextmanager
def resume_after(bng: BeamNGpy):
    try:
        yield
    finally:
        bng.control.resume()


def restart_vehicles(bng: BeamNGpy, *vehicles: Vehicle, pause: bool = True) -> None:
    bng.scenario.restart()
    if pause:
        bng.control.pause()
    else:
        bng.control.resume()
    for vehicle in vehicles:
        vehicle.set_shift_mode("arcade")
        vehicle.control(throttle=0.0, brake=0.0, steering=0.0)


@contextmanager
def loaded_scenario(
    beamng: BeamNGpy, *, name: str,
    vehicles: list[tuple[Vehicle, dict[str, Any]]],
    roads: list[Road] | None = None,
    level: str = TEST_LEVEL, hz: int = 60,
    description: str | None = None, label: str | None = None,
    pause: bool = False, precompile_shaders: bool = True,
    timeout: float = 60,
) -> Iterator[tuple[BeamNGpy, Scenario]]:
    """Load a scenario on the shared BeamNG instance and resume physics on exit."""
    with using_timeout(beamng, timeout) as bng:
        if not vehicles:
            raise ValueError("loaded_scenario requires at least one vehicle")

        scenario = Scenario(level, name, description=description)
        for vehicle, add_kwargs in vehicles:
            scenario.add_vehicle(vehicle, **add_kwargs)
        for road in roads or []:
            scenario.add_road(road)

        scenario.make(bng)
        bng.settings.set_deterministic(hz)
        if pause:
            bng.control.pause()
        bng.scenario.load(scenario, precompile_shaders=precompile_shaders)
        bng.ui.hide_hud()
        bng.scenario.start()
        for vehicle, _ in vehicles:
            vehicle.set_shift_mode("arcade")
        if not pause:
            bng.control.resume()

        if label:
            print(f"{label} start.")
        with resume_after(bng):
            yield bng, scenario
        if label:
            print(f"{label} complete.")


@contextmanager
def straight_road_scenario(
    beamng: BeamNGpy, *, name: str, label: str, vehicles: list[tuple[Vehicle, float]],
) -> Iterator[BeamNGpy]:
    """Paused tech_ground scenario with each vehicle at (0, y, 0) facing +Y on a 1.5 km straight road."""
    # Narrow single lane: span AI drives the road centre and follow AI the lane centre, which must match.
    road = Road("road_asphalt_2lane", rid="adas_test_road", default_width=4, drivability=1, one_way=True)
    road.add_nodes(*[(0.0, float(y), 0.0) for y in range(-50, 1550, 100)])
    with loaded_scenario(
        beamng, level="tech_ground", name=name, label=label,
        vehicles=[(vehicle, {"pos": (0, y, 0), "rot_quat": (0, 0, 1, 0)}) for vehicle, y in vehicles],
        roads=[road], precompile_shaders=False, pause=True,
    ) as (bng, _scenario):
        yield bng


def vehicle_state(vehicle: Vehicle) -> dict:
    vehicle.sensors.poll()
    return vehicle.sensors["state"]


def horizontal_speed(vehicle: Vehicle) -> float:
    vel = vehicle_state(vehicle)["vel"]
    return sqrt(vel[0] ** 2 + vel[1] ** 2)


def poll_auto_or_adhoc(sensor, is_auto: bool, *, attempt: int | None = None, adhoc_wait: float = 1):
    """Run automatic poll() or the ad-hoc request/collect flow."""
    suffix = f" {attempt}" if attempt is not None else ""
    if is_auto:
        print(f"\nAutomatic polling attempt{suffix}")
        return sensor.poll()

    print(f"\nAd-hoc poll request attempt{suffix}")
    request_id = sensor.send_ad_hoc_poll_request()
    print("Ad-hoc poll requests sent. Unique request Id number: ", request_id)
    sleep(adhoc_wait)
    print(
        "Is ad-hoc request complete? ",
        sensor.is_ad_hoc_poll_request_ready(request_id),
    )
    return sensor.collect_ad_hoc_poll_request(request_id)


def module_scene(
    name: str, description: str, label: str, *, color: str = "Red",
    obstacle_pos: tuple[float, float, float] | None = None,
    timeout: float = 300,
):
    @pytest.fixture(scope="module")
    def scene(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
        ego = Vehicle("ego_vehicle", model=TEST_VEHICLE_MODEL, license="PYTHON", color=color)
        if obstacle_pos is None:
            vehicles = [(ego, {})]
        else:
            other = Vehicle(OTHER_VEHICLE_ID, model=TEST_VEHICLE_MODEL, license="PYTHON", color="Blue")
            vehicles = [(ego, {}), (other, {"pos": obstacle_pos})]
        with loaded_scenario(
            beamng, name=name, description=description, vehicles=vehicles, label=label, timeout=timeout,
        ) as (bng, _scenario):
            yield bng, ego

    return scene


@contextmanager
def using_sensor(sensor, removed_msg: str | None = None):
    try:
        yield sensor
    finally:
        sensor.remove()
        if removed_msg:
            print(removed_msg)


def place_other_vehicle(
    bng: BeamNGpy, position: tuple[float, float, float], *, sim_steps: int = 60,
    settle_s: float = SETTLE_AFTER_TELEPORT_S,
) -> Vehicle:
    """Teleport the obstacle vehicle, advance the sim, then wait for physics to settle."""
    vehicle = bng.scenario.get_vehicle(OTHER_VEHICLE_ID)
    assert vehicle is not None, f"scenario vehicle {OTHER_VEHICLE_ID} not found"
    vehicle.teleport(position, reset=False)
    bng.control.step(sim_steps, wait=True)
    if settle_s > 0:
        sleep(settle_s)
    return vehicle


def assert_responds_to_motion(values: list[float], min_delta: float = 0.03, label: str = "values") -> None:
    """Scene motion should change readings; identical consecutive values indicate stale cache."""
    spread = max(values) - min(values)
    assert spread >= min_delta, (
        f"{label} did not change with scene motion: {values} (spread {spread:.4f})"
    )
    for i in range(1, len(values)):
        assert abs(values[i] - values[i - 1]) >= 1e-6, (
            f"{label} stale between motion steps: {values}"
        )


def cycle_visibility(sensor, toggles: int = 3, *, visualise: bool | None = None):
    if visualise is None:
        visualise = VISUALISE
    if not visualise:
        return
    for i in range(toggles * 2):
        sleep(1)
        sensor.set_is_visualised(not (i % 2 == 0))


def assert_vec3_close(actual, expected, tol: float = 1e-2):
    for a, e in zip(actual, expected):
        assert abs(a - e) <= tol, f"expected {expected}, got {actual}"


def assert_float_close(actual, expected, tol: float = 1e-3):
    assert abs(actual - expected) <= tol, f"expected {expected}, got {actual}"


def _normalized(v: tuple[float, float, float]) -> tuple[float, float, float]:
    length = sqrt(sum(c * c for c in v))
    if length == 0:
        return v
    return tuple(c / length for c in v)


# Getters whose values are unit vectors: setters normalize on the engine side, so
# compare against the normalized expected value rather than the raw input.
_UNIT_VECTOR_GETTERS = {"get_direction", "get_up"}


def exercise_roundtrip(sensor, cases: list[RoundtripCase] | None = None, *, update_time: float | None = None):
    """Assert getter/setter round-trips for each (getter_name, value, setter[, getter_kwargs]) tuple."""
    if cases is None:
        cases = [
            ("get_requested_update_time", update_time if update_time is not None else 0.2, sensor.set_requested_update_time),
        ]
    for case in cases:
        getter_name, value, setter = case[0], case[1], case[2]
        getter_kwargs = case[3] if len(case) > 3 else {}
        setter(value)
        actual = getattr(sensor, getter_name)(**getter_kwargs)
        if isinstance(value, tuple) and len(value) == 3:
            expected = _normalized(value) if getter_name in _UNIT_VECTOR_GETTERS else value
            assert_vec3_close(actual, expected)
        elif isinstance(value, tuple) and len(value) == 2:
            assert_float_close(actual[0], value[0])
            assert_float_close(actual[1], value[1])
        elif isinstance(value, bool):
            assert actual == value
        else:
            assert_float_close(float(actual), float(value))


@contextmanager
def using_timeout(
    beamng: BeamNGpy, timeout: float
) -> Iterator[BeamNGpy]:
    original_timeout = beamng.socket_timeout
    beamng.socket_timeout = timeout
    try:
        with beamng as bng:
            yield bng
    finally:
        beamng.socket_timeout = original_timeout
