from __future__ import annotations

import random

import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle, angle_to_quat
from beamngpy.sensors import Damage, Electrics, State


def test_electrics(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario("tech_ground", "electrics_test")
        vehicle = Vehicle("test_car", model="etk800")

        electrics = Electrics()
        vehicle.sensors.attach("electrics", electrics)

        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(beamng)

        bng.scenario.load(scenario)
        bng.scenario.start()
        bng.control.step(120)

        vehicle.control(throttle=1.0)

        bng.control.step(360)

        vehicle.sensors.poll()

    assert electrics["airspeed"] > 0
    assert electrics["wheelspeed"] > 0
    assert electrics["throttle_input"] > 0


def test_damage(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario("tech_ground", "damage_test")
        dummy = Vehicle("dummy", model="pickup")
        scenario.add_vehicle(dummy, pos=(0, 0, 0))
        scenario.make(beamng)

        vehicle = Vehicle("test_car", model="etk800")
        damage = Damage()
        vehicle.sensors.attach("damage", damage)

        bng.scenario.load(scenario)
        bng.scenario.start()

        scenario.add_vehicle(
            vehicle, pos=(0, 0, 32), rot_quat=angle_to_quat((-90, 0, 0)), cling=False
        )

        bng.control.step(600)

        vehicle.sensors.poll()

    assert damage["damage"] > 100


def test_state(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario("tech_ground", "state_test")
        vehicle = Vehicle("test_car", model="pickup")

        state = State()
        vehicle.sensors.attach("newstate", state)

        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(beamng)

        bng.scenario.load(scenario)
        bng.scenario.start()
        bng.control.step(20)

        vehicle.sensors.poll()

    assert state["pos"][0] < 0.1


if __name__ == "__main__":
    bng = BeamNGpy("localhost", 25252)
    test_electrics(bng)
