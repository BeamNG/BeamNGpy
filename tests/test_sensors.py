import random
import time

import numpy as np
import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.beamngcommon import BNGValueError, BNGError
from beamngpy.sensors import Camera, Lidar, Damage, Electrics, GForces, State
from beamngpy.sensors import IMU
from beamngpy.noise import RandomImageNoise


@pytest.fixture()
def beamng():
    beamng = BeamNGpy('localhost', 64256)
    return beamng


def assert_image_different(img):
    arr = np.array(img)
    arr = np.reshape(arr, arr.size)
    ref = arr[0]
    eq = arr[np.where(arr == ref)]
    assert eq.size != arr.size


def test_camera(beamng):
    scenario = Scenario('west_coast_usa', 'camera_test')
    vehicle = Vehicle('test_car', model='etk800')

    pos = (-0.3, 1, 1.0)
    direction = (0, 1, 0)
    fov = 120
    resolution = (64, 64)
    front_camera = Camera(pos, direction, fov, resolution,
                          colour=True, depth=True, annotation=True)
    vehicle.attach_sensor('front_cam', front_camera)

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=(0, 0, 45))
    scenario.make(beamng)

    with beamng as bng:
        bng.load_scenario(scenario)
        bng.step(120)
        time.sleep(20)

        sensors = bng.poll_sensors(vehicle)

        assert_image_different(sensors['front_cam']['colour'])
        assert_image_different(sensors['front_cam']['depth'])
        assert_image_different(sensors['front_cam']['annotation'])


def test_noise(beamng):
    scenario = Scenario('west_coast_usa', 'camera_test')
    vehicle = Vehicle('test_car', model='etk800')

    pos = (-0.3, 1, 1.0)
    direction = (0, 1, 0)
    fov = 120
    resolution = (64, 64)

    cam = Camera(pos, direction, fov, resolution, colour=True, depth=True)
    noise_cam = RandomImageNoise(cam)
    vehicle.attach_sensor('noise_cam', noise_cam)

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=(0, 0, 45))
    scenario.make(beamng)

    with beamng as bng:
        bng.load_scenario(scenario)
        bng.step(120)
        time.sleep(20)
        vehicle.poll_sensors()

        reference_img = np.array(noise_cam._sensor.data['colour'])
        noise_img = np.array(noise_cam.data['colour'])
        assert(not(np.array_equal(noise_img, reference_img)))


def test_lidar(beamng):
    scenario = Scenario('west_coast_usa', 'lidar_test')
    vehicle = Vehicle('test_car', model='etk800')

    lidar = Lidar()
    vehicle.attach_sensor('lidar', lidar)

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=(0, 0, 45))
    scenario.make(beamng)

    with beamng as bng:
        bng.load_scenario(scenario)
        bng.step(120)
        time.sleep(20)

        vehicle.poll_sensors()

        arr = lidar.data['points']
        ref = arr[0]
        eq = arr[np.where(arr == ref)]
        assert eq.size != arr.size


def test_gforces(beamng):
    scenario = Scenario('west_coast_usa', 'gforce_test')
    vehicle = Vehicle('test_car', model='etk800')

    gforces = GForces()
    vehicle.attach_sensor('gforces', gforces)

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=(0, 0, 45))
    scenario.make(beamng)

    gx = []
    gy = []
    with beamng as bng:
        bng.load_scenario(scenario)
        bng.start_scenario()
        bng.step(120)

        vehicle.ai_set_aggression(2)
        vehicle.ai_set_mode('span')

        for _ in range(64):
            bng.step(30)
            vehicle.poll_sensors()
            gx.append(gforces.data['gx'])
            gy.append(gforces.data['gy'])

    assert np.var(gx) > 1 and np.var(gy) > 1


def test_electrics(beamng):
    scenario = Scenario('smallgrid', 'electrics_test')
    vehicle = Vehicle('test_car', model='etk800')

    electrics = Electrics()
    vehicle.attach_sensor('electrics', electrics)

    scenario.add_vehicle(vehicle, pos=(0, 0, 0))
    scenario.make(beamng)

    with beamng as bng:
        bng.load_scenario(scenario)
        bng.start_scenario()
        bng.step(120)

        vehicle.control(throttle=1.0)

        bng.step(360)

        vehicle.poll_sensors()

    assert electrics.data['airspeed'] > 0
    assert electrics.data['wheelspeed'] > 0
    assert electrics.data['throttle_input'] > 0


def test_damage(beamng):
    scenario = Scenario('smallgrid', 'damage_test')
    dummy = Vehicle('dummy', model='pickup')
    scenario.add_vehicle(dummy, pos=(0, 0, 0))
    scenario.make(beamng)

    vehicle = Vehicle('test_car', model='etk800')
    damage = Damage()
    vehicle.attach_sensor('damage', damage)

    with beamng as bng:
        bng.load_scenario(scenario)
        bng.start_scenario()

        scenario.add_vehicle(vehicle, pos=(0, 0, 32), rot=(-90, 0, 0),
                             cling=False)

        bng.step(600)

        vehicle.poll_sensors()

    assert damage.data['damage'] > 100


def test_state(beamng):
    scenario = Scenario('smallgrid', 'vehicle_state_test')
    vehicle = Vehicle('test_car', model='pickup')

    state = State()
    vehicle.attach_sensor('state', state)

    scenario.add_vehicle(vehicle, pos=(0, 0, 0))
    scenario.make(beamng)

    with beamng as bng:
        bng.load_scenario(scenario)
        bng.start_scenario()
        bng.step(20)

        vehicle.poll_sensors()

    assert state.data['pos'][0] < 0.1


def test_imu(beamng):
    scenario = Scenario('smallgrid', 'vehicle_state_test')
    vehicle = Vehicle('test_car', model='etk800')

    imu_pos = IMU(pos=(0.73, 0.51, 0.8), debug=True)
    imu_node = IMU(node=0, debug=True)
    vehicle.attach_sensor('imu_pos', imu_pos)
    vehicle.attach_sensor('imu_node', imu_node)

    scenario.add_vehicle(vehicle, pos=(0, 0, 0))
    scenario.make(beamng)

    with beamng as bng:
        bng.load_scenario(scenario)
        bng.start_scenario()
        bng.step(20)

        pax, pay, paz, pgx, pgy, pgz = [], [], [], [], [], []

        for _ in range(30):
            # Stand still, sample IMU
            bng.step(60)
            vehicle.poll_sensors()
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
            bng.step(60)
            vehicle.poll_sensors()
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

        for arr in [pax, pay, paz, pgx, pgy, pgz]:
            assert np.max(arr) > 0.01
            assert np.min(arr) < -0.01

        # See if IMU at different position ended up with different measurements
        for parr, narr in zip([pax, pay, paz, pgx, pgy, pgz],
                              [nax, nay, naz, ngx, ngy, ngz]):
            assert np.mean(parr) != np.mean(narr)


if __name__ == '__main__':
    bng = BeamNGpy('localhost', 64256)
    test_camera(bng)
