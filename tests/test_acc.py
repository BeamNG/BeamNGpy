from __future__ import annotations

from math import dist
from typing import Iterator

import pytest

from beamngpy import BeamNGpy, Vehicle

from tests.helpers import (
    TEST_ROAD_END_NODE,
    horizontal_speed,
    restart_vehicles,
    straight_road_scenario,
    vehicle_state,
)

LEAD_SPEED = 10.0
ACC_SPEED = 20.0
REDUCED_ACC_SPEED = 5.0
SPEED_TOLERANCE = 1.5
# Centre-to-centre distance below which two etk800s are touching.
MIN_CENTER_GAP = 6.0


@pytest.fixture(scope="module")
def acc_scene(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle, Vehicle]]:
    ego = Vehicle("ego_vehicle", model="etk800", license="EGO", color="Red")
    lead = Vehicle("other_vehicle", model="etk800", license="OTHER", color="Blue")
    with straight_road_scenario(
        beamng, name="acc_test", label="ACC test", vehicles=[(ego, 0.0), (lead, 15.0)],
    ) as bng:
        yield bng, ego, lead


@pytest.fixture(autouse=True)
def _reset_shared_scene(acc_scene):
    bng, ego, lead = acc_scene
    restart_vehicles(bng, ego, lead)
    yield
    ego.acc.stop()


def _gap(a: Vehicle, b: Vehicle) -> float:
    return dist(vehicle_state(a)["pos"], vehicle_state(b)["pos"])


def _start_following(ego: Vehicle, lead: Vehicle) -> None:
    # Traffic mode does not drive on the dead-end test road; the ego AI only steers while ACC owns its pedals.
    lead.ai.set_waypoint(TEST_ROAD_END_NODE)
    lead.ai.set_speed(LEAD_SPEED, "limit")
    ego.ai.set_mode("span")
    ego.acc.start(ACC_SPEED)


def _drive(bng: BeamNGpy, ego: Vehicle, lead: Vehicle, seconds: int) -> list[float]:
    """Step one second at a time and return the gap after each step."""
    gaps = []
    for _ in range(seconds):
        bng.control.step(60, wait=True)
        gaps.append(_gap(ego, lead))
    return gaps


def test_acc_follows_lead(acc_scene):
    bng, ego, lead = acc_scene
    start = vehicle_state(ego)["pos"]
    _start_following(ego, lead)
    gaps = _drive(bng, ego, lead, 30)

    travelled = dist(vehicle_state(ego)["pos"], start)
    speed = horizontal_speed(ego)
    print(f"ego travelled {travelled:.1f} m, speed {speed:.2f} m/s, min gap {min(gaps):.1f} m")
    assert travelled > 50.0
    assert min(gaps) > MIN_CENTER_GAP, f"ego got too close to the lead: {gaps}"
    assert speed < LEAD_SPEED + 2.0, "ego should be capped by the lead, not the ACC target"


def test_acc_change_speed(acc_scene):
    bng, ego, lead = acc_scene
    _start_following(ego, lead)
    _drive(bng, ego, lead, 15)
    gap_before = _gap(ego, lead)

    ego.acc.change_speed(REDUCED_ACC_SPEED)
    gaps = _drive(bng, ego, lead, 20)

    speed = horizontal_speed(ego)
    print(f"ego speed {speed:.2f} m/s, gap {gap_before:.1f} -> {gaps[-1]:.1f} m")
    assert speed < REDUCED_ACC_SPEED + SPEED_TOLERANCE
    assert gaps[-1] > gap_before


def test_acc_stop(acc_scene):
    bng, ego, _lead = acc_scene
    ego.acc.start(ACC_SPEED)
    bng.control.step(60, wait=True)
    ego.acc.stop()
    ego.acc.stop()
