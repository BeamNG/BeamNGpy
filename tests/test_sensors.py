from __future__ import annotations

import random

import numpy as np
import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle, angle_to_quat
from beamngpy.sensors import IMU, Damage, Electrics, State


@pytest.fixture()
def beamng():
    beamng = BeamNGpy('localhost', 64256, quit_on_close=False)
    return beamng


def test_electrics(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario('smallgrid', 'electrics_test')
        vehicle = Vehicle('test_car', model='etk800')

        electrics = Electrics()
        vehicle.sensors.attach('electrics', electrics)

        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(beamng)

        bng.scenario.load(scenario)
        bng.scenario.start()
        bng.control.step(120)

        vehicle.control(throttle=1.0)

        bng.control.step(360)

        vehicle.sensors.poll()

    assert electrics['airspeed'] > 0
    assert electrics['wheelspeed'] > 0
    assert electrics['throttle_input'] > 0


def test_damage(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario('smallgrid', 'damage_test')
        dummy = Vehicle('dummy', model='pickup')
        scenario.add_vehicle(dummy, pos=(0, 0, 0))
        scenario.make(beamng)

        vehicle = Vehicle('test_car', model='etk800')
        damage = Damage()
        vehicle.sensors.attach('damage', damage)

        bng.scenario.load(scenario)
        bng.scenario.start()

        scenario.add_vehicle(vehicle, pos=(0, 0, 32), rot_quat=angle_to_quat((-90, 0, 0)), cling=False)

        bng.control.step(600)

        vehicle.sensors.poll()

    assert damage['damage'] > 100


def test_state(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario('smallgrid', 'vehicle_state_test')
        vehicle = Vehicle('test_car', model='pickup')

        state = State()
        vehicle.sensors.attach('newstate', state)

        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(beamng)

        bng.scenario.load(scenario)
        bng.scenario.start()
        bng.control.step(20)

        vehicle.sensors.poll()

    assert state.data['pos'][0] < 0.1


def test_imu(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario('smallgrid', 'vehicle_state_test')
        vehicle = Vehicle('test_car', model='etk800')

        imu_pos = IMU(pos=(0.73, 0.51, 0.8), debug=True)
        imu_node = IMU(node=0, debug=True)
        vehicle.sensors.attach('imu_pos', imu_pos)
        vehicle.sensors.attach('imu_node', imu_node)

        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(beamng)

        bng.scenario.load(scenario)
        bng.scenario.start()
        bng.control.step(20)

        pax, pay, paz, pgx, pgy, pgz = [], [], [], [], [], []

        for _ in range(30):
            # Stand still, sample IMU
            bng.control.step(60)
            vehicle.sensors.poll()
            pax.append(imu_pos.data['aX'])
            pay.append(imu_pos.data['aY'])
            paz.append(imu_pos.data['aZ'])
            pgx.append(imu_pos.data['gX'])
            pgy.append(imu_pos.data['gY'])
            pgz.append(imu_pos.data['gZ'])

        # Some slight movement is bound to happen since the engine is one and
        # the vehicle isn't perfectly stable; hence no check for == 0
        assert np.mean(pax) < 1
        assert np.mean(pay) < 1
        assert np.mean(paz) < 1
        assert np.mean(pgx) < 1
        assert np.mean(pgy) < 1
        assert np.mean(pgz) < 1

        pax, pay, paz, pgx, pgy, pgz = [], [], [], [], [], []
        nax, nay, naz, ngx, ngy, ngz = [], [], [], [], [], []

        for _ in range(30):
            # Drive randomly, sample IMU
            t = random.random() * 2 - 1
            s = random.random() * 2 - 1
            vehicle.control(throttle=t, steering=s)
            bng.control.step(60)
            vehicle.sensors.poll()
            pax.append(imu_pos.data['aX'])
            pay.append(imu_pos.data['aY'])
            paz.append(imu_pos.data['aZ'])
            pgx.append(imu_pos.data['gX'])
            pgy.append(imu_pos.data['gY'])
            pgz.append(imu_pos.data['gZ'])

            nax.append(imu_node.data['aX'])
            nay.append(imu_node.data['aY'])
            naz.append(imu_node.data['aZ'])
            ngx.append(imu_node.data['gX'])
            ngy.append(imu_node.data['gY'])
            ngz.append(imu_node.data['gZ'])

        for arr in [pax, pay, pgx, pgy, pgz]:
            assert np.max(arr) > 0.01
            assert np.min(arr) < -0.01

        # See if IMU at different position ended up with different measurements
        for parr, narr in zip([pax, pay, paz, pgx, pgy, pgz],
                              [nax, nay, naz, ngx, ngy, ngz]):
            assert np.mean(parr) != np.mean(narr)

if __name__ == '__main__':
    bng = BeamNGpy('localhost', 64256)
    test_imu(bng)
