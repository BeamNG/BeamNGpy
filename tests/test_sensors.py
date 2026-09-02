from __future__ import annotations

from contextlib import contextmanager
from typing import Iterator

import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle, angle_to_quat
from beamngpy.sensors import Damage, Electrics, State


@contextmanager
def sensors_scenario(
    beamng: BeamNGpy,
) -> Iterator[tuple[BeamNGpy, Vehicle, Electrics, Damage, State]]:
    with beamng as bng:
        scenario = Scenario("tech_ground", "sensors_test")
        vehicle = Vehicle("test_car", model="etk800")

        electrics = Electrics()
        damage = Damage()
        state = State()
        vehicle.sensors.attach("electrics", electrics)
        vehicle.sensors.attach("damage", damage)
        vehicle.sensors.attach("newstate", state)

        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(bng)

        bng.settings.set_deterministic(60)
        bng.control.pause()
        bng.scenario.load(scenario)
        bng.scenario.start()

        yield bng, vehicle, electrics, damage, state


@pytest.fixture(scope="module")
def sensors_scene(
    beamng: BeamNGpy,
) -> Iterator[tuple[BeamNGpy, Vehicle, Electrics, Damage, State]]:
    with sensors_scenario(beamng) as scene:
        yield scene


def _reset_scene(bng: BeamNGpy, vehicle: Vehicle) -> None:
    bng.scenario.restart()
    bng.control.pause()
    vehicle.control(throttle=0.0, brake=0.0, steering=0.0)


def test_electrics(sensors_scene):
    bng, vehicle, electrics, _, _ = sensors_scene
    _reset_scene(bng, vehicle)

    bng.control.step(120)
    vehicle.control(throttle=1.0)
    bng.control.step(360)
    vehicle.sensors.poll()

    assert electrics["airspeed"] > 0
    assert electrics["wheelspeed"] > 0
    assert electrics["throttle_input"] > 0


def test_damage(sensors_scene):
    bng, vehicle, _, damage, _ = sensors_scene
    _reset_scene(bng, vehicle)

    # Drop the shared vehicle instead of building a separate crash scenario.
    vehicle.teleport(
        (0, 0, 32), rot_quat=angle_to_quat((-90, 0, 0)), reset=True
    )
    bng.control.step(600)
    vehicle.sensors.poll()

    assert damage["damage"] > 100


def test_state(sensors_scene):
    bng, vehicle, _, _, state = sensors_scene
    _reset_scene(bng, vehicle)

    bng.control.step(20)
    vehicle.sensors.poll()

    assert state["pos"][0] < 0.1


if __name__ == "__main__":
    bng = BeamNGpy("localhost", 25252, quit_on_close=False)
    with sensors_scenario(bng) as scene:
        for test in (test_electrics, test_damage, test_state):
            test(scene)
