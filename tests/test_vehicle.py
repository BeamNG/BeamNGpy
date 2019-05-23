import numpy as np
import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.beamngcommon import BNGValueError


@pytest.fixture()
def beamng():
    beamng = BeamNGpy('localhost', 64256)
    return beamng


def assert_continued_movement(bng, vehicle, start_pos):
    last_pos = start_pos
    for _ in range(5):
        bng.step(60)
        vehicle.update_vehicle()
        assert np.linalg.norm(np.array(vehicle.state['pos']) - last_pos) > 1
        last_pos = vehicle.state['pos']


def test_vehicle_move(beamng):
    with beamng as bng:
        bng.set_deterministic()

        scenario = Scenario('smallgrid', 'move_test')
        vehicle = Vehicle('test_car', model='etk800')
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot=(0, 0, 0))
        scenario.make(bng)
        bng.load_scenario(scenario)
        bng.start_scenario()
        bng.pause()
        vehicle.control(throttle=1)
        bng.step(120)
        vehicle.update_vehicle()
        assert np.linalg.norm(vehicle.state['pos']) > 1

    scenario.delete(beamng)


def test_vehicle_ai(beamng):
    with beamng as bng:
        bng.set_deterministic()

        scenario = Scenario('west_coast_usa', 'ai_test')
        vehicle = Vehicle('test_car', model='etk800')
        other = Vehicle('other', model='etk800')
        pos = [-717.121, 101, 118.675]
        scenario.add_vehicle(vehicle, pos=pos, rot=(0, 0, 45))
        scenario.add_vehicle(other, pos=(-453, 700, 75), rot=(0, 0, 45))
        scenario.make(bng)

        bng.load_scenario(scenario)

        bng.start_scenario()
        bng.pause()

        vehicle.ai_set_mode('span')
        assert_continued_movement(bng, vehicle, pos)

        bng.restart_scenario()
        bng.pause()

        vehicle.ai_set_waypoint('Bridge3_B')
        assert_continued_movement(bng, vehicle, pos)

        bng.restart_scenario()
        bng.pause()

        vehicle.ai_set_target('other', mode='chase')
        assert_continued_movement(bng, vehicle, pos)

        bng.restart_scenario()
        bng.pause()

        vehicle.ai_set_target('other', mode='flee')
        assert_continued_movement(bng, vehicle, pos)

    scenario.delete(beamng)


def test_vehicle_spawn(beamng):
    scenario = Scenario('smallgrid', 'spawn_test')
    vehicle = Vehicle('irrelevant', model='pickup')
    scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot=(0, 0, 0))
    scenario.make(beamng)

    with beamng as bng:
        bng.load_scenario(scenario)
        bng.start_scenario()

        other = Vehicle('relevant', model='etk800')
        scenario.add_vehicle(other, pos=(10, 10, 0), rot=(0, 0, 0))
        other.update_vehicle()
        assert 'pos' in other.state
        bng.step(120)
        scenario.remove_vehicle(other)
        bng.step(600)
        assert other.state is None
