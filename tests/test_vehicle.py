import itertools
import numpy as np
import pytest
import time

from beamngpy import BeamNGpy, Scenario, sensors, Vehicle, setup_logging
from beamngpy.beamngcommon import BNGValueError


@pytest.fixture()
def beamng():
    beamng = BeamNGpy('localhost', 64256)
    return beamng


def test_get_available_vehicles(beamng):
    with beamng as bng:
        resp = bng.get_available_vehicles()
        assert len(resp) > 0


def assert_continued_movement(bng, vehicle, start_pos):
    last_pos = start_pos
    for _ in range(5):
        bng.step(120, wait=True)
        vehicle.update_vehicle()
        assert np.linalg.norm(np.array(vehicle.state['pos']) - last_pos) > 0.5
        last_pos = vehicle.state['pos']


def assert_non_movement(bng, vehicle, start_pos):
    last_pos = start_pos
    for _ in range(5):
        bng.step(60, wait=True)
        vehicle.update_vehicle()
        assert np.linalg.norm(np.array(vehicle.state['pos']) - last_pos) < 0.5
        last_pos = vehicle.state['pos']


def test_vehicle_move(beamng):
    with beamng as bng:
        bng.set_steps_per_second(50)
        bng.set_deterministic()

        scenario = Scenario('smallgrid', 'move_test')
        vehicle = Vehicle('test_car', model='etk800')
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot=(0, 0, 0))
        scenario.make(bng)
        bng.load_scenario(scenario)
        bng.start_scenario()
        bng.pause()
        vehicle.control(throttle=1)
        bng.step(120, wait=True)
        vehicle.update_vehicle()
        assert np.linalg.norm(vehicle.state['pos']) > 1

    scenario.delete(beamng)


def test_vehicle_ai(beamng):
    with beamng as bng:
        bng.set_steps_per_second(50)
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

        vehicle.ai_set_waypoint('Bridge4_B')
        assert_continued_movement(bng, vehicle, pos)

        bng.restart_scenario()
        bng.pause()

        vehicle.ai_set_target('other', mode='chase')
        assert_continued_movement(bng, vehicle, pos)

        bng.restart_scenario()
        bng.pause()

        vehicle.ai_set_target('other', mode='flee')
        assert_continued_movement(bng, vehicle, pos)

        bng.restart_scenario()
        bng.pause()

        script = [
            {'x': -735, 'y': 86.7, 'z': 119, 't': 0},
            {'x': -752, 'y': 70, 'z': 119, 't': 5},
            {'x': -762, 'y': 60, 'z': 119, 't': 8},
        ]
        vehicle.ai_set_script(script)
        bng.step(600, wait=True)
        vehicle.update_vehicle()
        ref = [script[1]['x'], script[1]['y'], script[1]['z']]
        pos = vehicle.state['pos']
        ref, pos = np.array(ref), np.array(pos)
        assert np.linalg.norm(ref - pos) < 2.5

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
        bng.step(120, wait=True)
        scenario.remove_vehicle(other)
        bng.step(600, wait=True)
        assert other.state is None


def test_vehicle_bbox(beamng):
    scenario = Scenario('west_coast_usa', 'bbox_test')
    vehicle_a = Vehicle('vehicle_a', model='etk800')
    vehicle_b = Vehicle('vehicle_b', model='etk800')
    pos = [-717.121, 101, 118.675]
    scenario.add_vehicle(vehicle_a, pos=pos, rot=(0, 0, 45))
    pos = [-453, 700, 75]
    scenario.add_vehicle(vehicle_b, pos=pos, rot=(0, 0, 45))
    scenario.make(beamng)

    with beamng as bng:
        bng.load_scenario(scenario)
        bng.start_scenario()
        bng.pause()

        bbox_beg = bng.get_vehicle_bbox(vehicle_a)
        vehicle_a.ai_set_mode('span')
        bng.step(2000, wait=True)
        bbox_end = bng.get_vehicle_bbox(vehicle_a)

        for k, v in bbox_beg.items():
            assert k in bbox_end
            assert v != bbox_end[k]


def _get_matching_entries(a, b):
    ret = {}
    for k, _ in a.items():
        ret[k] = b[k]
    return ret


def _check_lights(target, data, msg_fmt):
    data = _get_matching_entries(target, data)

    for l, v in target.items():
        msg = msg_fmt.format(target, l)
        assert v == data[l], msg


def test_lights(beamng):
    scenario = Scenario('smallgrid', 'bbox_test')
    config = 'vehicles/etk800/police.pc'
    vehicle = Vehicle('vehicle', model='etk800', partConfig=config)
    other = Vehicle('other', model='pickup')
    electrics = sensors.Electrics()
    vehicle.attach_sensor('electrics', electrics)
    electrics = sensors.Electrics()
    other.attach_sensor('electrics', electrics)
    scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot=(0, 0, 0))
    scenario.add_vehicle(other, pos=(10, 10, 0), rot=(0, 0, 0))
    scenario.make(beamng)

    binary = {'left_signal', 'right_signal', 'hazard_signal'}
    ternary = {'headlights', 'fog_lights', 'lightbar'}

    all_off = {}
    possible = []
    for light in binary:
        all_off[light] = False
        possible.append((light, False))
        possible.append((light, True))
    for light in ternary:
        all_off[light] = 0
        for i in range(3):
            possible.append((light, i))

    with beamng as bng:
        bng.load_scenario(scenario)
        bng.start_scenario()
        bng.pause()

        for r in range(len(binary) + len(ternary)):
            r = r + 1
            for combo in itertools.combinations(possible, r):
                vals = {}
                for light, value in combo:
                    vals[light] = value

                expected = dict(**vals)

                if 'hazard_signal' in expected and expected['hazard_signal']:
                    # left/right signals are overridden by the hazard light
                    # so we skip tests that set both
                    if 'left_signal' in vals:
                        continue
                    if 'right_signal' in vals:
                        continue
                else:
                    # Signals negate each other so we skip tests that set both
                    if 'left_signal' in vals and vals['left_signal']:
                        if 'right_signal' in vals and vals['right_signal']:
                            continue
                    if 'right_signal' in vals and vals['right_signal']:
                        if 'left_signal' in vals and vals['left_signal']:
                            continue

                # bng.step(1, wait=True)
                vehicle.set_lights(**vals)
                bng.step(1, wait=True)
                data = bng.poll_sensors(vehicle)['electrics']
                msg_fmt = 'Setting the combination of {} did not result in ' \
                          'corresponding light states in the case of {}.'
                _check_lights(expected, data, msg_fmt)

                data = bng.poll_sensors(other)['electrics']
                msg_fmt = 'Other vehicle has lights on when it should not. ' \
                          'Expected {} but got mismatch in the case of: {}'
                _check_lights(all_off, data, msg_fmt)

                vehicle.set_lights(**all_off)
                bng.step(1, wait=True)
                data = bng.poll_sensors(vehicle)['electrics']
                msg_fmt = 'Lights did not turn off correctly. Expected {} ' \
                          'but got mismatch in the case of: {}'
                _check_lights(all_off, data, msg_fmt)


def test_traffic(beamng):
    with beamng as bng:
        bng.set_steps_per_second(50)
        bng.set_deterministic()

        scenario = Scenario('west_coast_usa', 'ai_test')
        vehicle = Vehicle('ego', model='etk800')
        other = Vehicle('traffic', model='etk800')
        pos = [-717.121, 101, 118.675]
        scenario.add_vehicle(vehicle, pos=pos, rot=(0, 0, 45))
        pos = [-453, 700, 75]
        scenario.add_vehicle(other, pos=pos, rot=(0, 0, 45))
        scenario.make(bng)

        bng.load_scenario(scenario)
        bng.start_scenario()
        bng.pause()

        bng.start_traffic([other])
        bng.switch_vehicle(other)

        bng.step(300, wait=True)  # Give vehicle ~5seconds to start

        assert_continued_movement(bng, other, pos)
