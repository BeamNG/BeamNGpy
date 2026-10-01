from __future__ import annotations

from typing import Iterator

import pytest

from beamngpy import BeamNGpy, Vehicle, angle_to_quat
from beamngpy.sensors import Damage, Electrics, State

from tests.helpers import loaded_scenario, restart_vehicles


def _attached_sensors() -> tuple[Vehicle, Electrics, Damage, State]:
    vehicle = Vehicle("test_car", model="etk800")
    electrics = Electrics()
    damage = Damage()
    state = State()
    vehicle.sensors.attach("electrics", electrics)
    vehicle.sensors.attach("damage", damage)
    vehicle.sensors.attach("newstate", state)
    return vehicle, electrics, damage, state


@pytest.fixture(scope="module")
def sensors_scene(
    beamng: BeamNGpy,
) -> Iterator[tuple[BeamNGpy, Vehicle, Electrics, Damage, State]]:
    vehicle, electrics, damage, state = _attached_sensors()
    with loaded_scenario(
        beamng, level="tech_ground", name="sensors_test",
        vehicles=[(vehicle, {"pos": (0, 0, 0)})],
        pause=True,
    ) as (bng, _scenario):
        yield bng, vehicle, electrics, damage, state


@pytest.fixture(autouse=True)
def _reset_shared_scene(sensors_scene):
    bng, vehicle, *_ = sensors_scene
    restart_vehicles(bng, vehicle)


def test_electrics(sensors_scene):
    bng, vehicle, electrics, _, _ = sensors_scene

    bng.control.step(120)
    vehicle.control(throttle=1.0)
    bng.control.step(360)
    vehicle.sensors.poll()

    assert electrics["airspeed"] > 0
    assert electrics["wheelspeed"] > 0
    assert electrics["throttle_input"] > 0


def test_damage(sensors_scene):
    bng, vehicle, _, damage, _ = sensors_scene

    # Drop the shared vehicle instead of building a separate crash scenario.
    vehicle.teleport(
        (0, 0, 32), rot_quat=angle_to_quat((-90, 0, 0)), reset=True
    )
    bng.control.step(600)
    vehicle.sensors.poll()

    assert damage["damage"] > 100


def test_state(sensors_scene):
    bng, vehicle, _, _, state = sensors_scene

    bng.control.step(20)
    vehicle.sensors.poll()

    assert state["pos"][0] < 0.1

