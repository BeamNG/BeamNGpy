from __future__ import annotations

from contextlib import suppress
from math import dist
from typing import Iterator

import pytest

from beamngpy import BeamNGpy, Vehicle
from beamngpy.logging import BNGError

from tests.helpers import horizontal_speed, restart_vehicles, straight_road_scenario, vehicle_state

# Traffic mode does not drive on the dead-end test road; span does.
SPAN_MODE = 1
PLATOON_SPEED = 15.0
REDUCED_SPEED = 5.0
SPEED_TOLERANCE = 1.5
FOLLOWER_SPEED_TOLERANCE = 2.0
# Centre-to-centre distance band between adjacent bastions in formation.
MIN_CENTER_GAP = 6.0
MAX_CENTER_GAP = 25.0

COLUMN = [
    ("leader", "Orange", 36.0),
    ("follower1", "Purple", 24.0),
    ("follower2", "Yellow", 12.0),
    ("follower3", "Black", 0.0),
]


@pytest.fixture(scope="module")
def platoon_scene(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, list[Vehicle]]]:
    vehicles = [
        Vehicle(name, model="bastion", license=name, color=color) for name, color, _ in COLUMN
    ]
    with straight_road_scenario(
        beamng, name="platooning_test", label="Platooning test",
        vehicles=[(vehicle, y) for vehicle, (_, _, y) in zip(vehicles, COLUMN)],
    ) as bng:
        yield bng, vehicles


@pytest.fixture(autouse=True)
def platoon_ids(platoon_scene) -> Iterator[list[int]]:
    bng, vehicles = platoon_scene
    restart_vehicles(bng, *vehicles)
    ids: list[int] = []
    yield ids
    for platoon_id in ids:
        with suppress(BNGError):
            bng.platoon.disband(platoon_id)


def _build_platoon(bng: BeamNGpy, vehicles: list[Vehicle], ids: list[int]) -> int:
    leader, f1, f2, f3 = vehicles
    platoon_id = bng.platoon.create(leader, f1)
    ids.append(platoon_id)
    bng.platoon.join(platoon_id, f2, index=2)
    bng.platoon.join(platoon_id, f3)
    return platoon_id


def _drive(bng: BeamNGpy, seconds: int) -> None:
    for _ in range(seconds):
        bng.control.step(60, wait=True)


def test_platoon_drives_in_formation(platoon_scene, platoon_ids):
    bng, vehicles = platoon_scene
    starts = [vehicle_state(v)["pos"] for v in vehicles]
    platoon_id = _build_platoon(bng, vehicles, platoon_ids)
    bng.platoon.launch(platoon_id, SPAN_MODE, PLATOON_SPEED)
    _drive(bng, 30)

    positions = [vehicle_state(v)["pos"] for v in vehicles]
    speeds = [horizontal_speed(v) for v in vehicles]
    gaps = [dist(a, b) for a, b in zip(positions, positions[1:])]
    print(f"speeds {[round(s, 2) for s in speeds]}, gaps {[round(g, 1) for g in gaps]}")

    for vehicle, start, pos in zip(vehicles, starts, positions):
        assert dist(start, pos) > 50.0, f"{vehicle.vid} did not drive with the platoon"
    for gap in gaps:
        assert MIN_CENTER_GAP < gap < MAX_CENTER_GAP, f"platoon lost formation: {gaps}"
    for vehicle, speed in zip(vehicles[1:], speeds[1:]):
        assert abs(speed - speeds[0]) < FOLLOWER_SPEED_TOLERANCE, (
            f"{vehicle.vid} speed {speed:.2f} does not match leader {speeds[0]:.2f}"
        )


def test_platoon_change_speed(platoon_scene, platoon_ids):
    bng, vehicles = platoon_scene
    platoon_id = _build_platoon(bng, vehicles, platoon_ids)
    bng.platoon.launch(platoon_id, SPAN_MODE, PLATOON_SPEED)
    _drive(bng, 15)

    bng.platoon.change_speed(platoon_id, REDUCED_SPEED)
    _drive(bng, 20)

    speeds = [horizontal_speed(v) for v in vehicles]
    print(f"speeds after change {[round(s, 2) for s in speeds]}")
    for vehicle, speed in zip(vehicles, speeds):
        assert speed < REDUCED_SPEED + SPEED_TOLERANCE, f"{vehicle.vid} did not slow down: {speed:.2f}"


def test_platoon_split_leave_disband(platoon_scene, platoon_ids):
    bng, vehicles = platoon_scene
    platoon_id = _build_platoon(bng, vehicles, platoon_ids)

    rear_id = bng.platoon.split(platoon_id, 2)
    platoon_ids.append(rear_id)
    assert isinstance(rear_id, int)
    assert rear_id != platoon_id

    bng.platoon.leave(rear_id, vehicles[3])
    bng.platoon.disband(platoon_id)


def test_platoon_invalid_requests(platoon_scene, platoon_ids):
    bng, vehicles = platoon_scene
    leader, f1, f2, f3 = vehicles

    with pytest.raises(BNGError):
        bng.platoon.create(leader, leader)

    platoon_id = bng.platoon.create(leader, f1)
    platoon_ids.append(platoon_id)
    bng.platoon.join(platoon_id, f2)
    with pytest.raises(BNGError):
        bng.platoon.split(platoon_id, 2)

    bng.platoon.join(platoon_id, f3)
    with pytest.raises(BNGError):
        bng.platoon.split(platoon_id, 1)
