import random
import time

import numpy as np
import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging, ProceduralCube
from beamngpy.beamngcommon import BNGValueError, BNGError
from beamngpy.sensors import Camera, Lidar, Damage, Electrics, GForces, State
from beamngpy.sensors import IMU, Ultrasonic
from beamngpy.noise import RandomImageNoise, RandomLIDARNoise


SHMEM_OPTIONS = [False, True]
SHMEM_IDS = ['Socket', 'Shmem']


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


@pytest.mark.parametrize('shmem', SHMEM_OPTIONS, ids=SHMEM_IDS)
def test_camera(beamng, shmem):
    with beamng as bng:
        scenario = Scenario('west_coast_usa', 'camera_test')
        vehicle = Vehicle('test_car', model='etk800')

        pos = (-0.3, 1, 1.0)
        direction = (0, 1, 0)
        fov = 120
        resolution = (64, 64)
        front_camera = Camera(pos, direction, fov, resolution, colour=True,
                              depth=True, annotation=True, instance=True,
                              shmem=shmem)
        vehicle.attach_sensor('front_cam', front_camera)

        scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675),
                             rot=(0, 0, 45))
        scenario.make(beamng)

        bng.load_scenario(scenario)
        bng.start_scenario()
        bng.pause()
        bng.step(120)
        time.sleep(20)

        sensors = bng.poll_sensors(vehicle)

        assert_image_different(sensors['front_cam']['colour'])
        assert_image_different(sensors['front_cam']['depth'])
        assert_image_different(sensors['front_cam']['annotation'])
        assert_image_different(sensors['front_cam']['instance'])
        annotation = sensors['front_cam']['annotation']
        instance = sensors['front_cam']['instance']
        assert not (np.array(annotation) == np.array(instance)).all()


def test_multicam(beamng):
    with beamng as bng:
        scenario = Scenario('west_coast_usa', 'camera_test')
        vehicle = Vehicle('test_car', model='etk800')

        pos = (-0.3, 1, 1.0)
        direction = (0, 1, 0)
        fov = 120
        resolution = (64, 64)
        front_cam1 = Camera(pos, direction, fov, resolution, colour=True,
                            depth=True, annotation=True, instance=True,
                            shmem=True)
        vehicle.attach_sensor('front_cam1', front_cam1)
        front_cam2 = Camera(pos, direction, fov, resolution, colour=True,
                            depth=True, annotation=True, instance=True,
                            shmem=True)
        vehicle.attach_sensor('front_cam2', front_cam2)

        scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675),
                             rot=(0, 0, 45))
        scenario.make(beamng)

        bng.load_scenario(scenario)
        bng.start_scenario()
        bng.pause()
        bng.step(120)
        time.sleep(20)

        vehicle.poll_sensors()


def test_bboxes(beamng):
    with beamng as bng:
        scenario = Scenario('west_coast_usa', 'bbox_test')

        ego = Vehicle('ego', model='etk800', color='White')
        scenario.add_vehicle(ego, pos=(-725.365, 92.4684, 118.437),
                             rot_quat=(-0.006, -0.0076, 0.921, -0.389))

        camera = Camera((-0.3, 1, 1), (0, 1, 0), 75, (1024, 1024), colour=True,
                        depth=True, annotation=True, instance=True)
        ego.attach_sensor('camera', camera)

        car1 = Vehicle('car1', model='etk800', color='Green')
        scenario.add_vehicle(car1, pos=(-710.76, 101.50, 118.56),
                             rot_quat=(-0.006, -0.0076, 0.921, -0.389))

        car2 = Vehicle('car2', model='etk800', color='Red')
        scenario.add_vehicle(car2, pos=(-715.83, 96.69, 118.53),
                             rot_quat=(-0.006, -0.0076, 0.921, -0.389))

        car3 = Vehicle('car3', model='etki', color='Blue')
        scenario.add_vehicle(car3, pos=(-696.96, 126.9, 118.44),
                             rot_quat=(0.0181, -0.0065, 0.3816, 0.924))

        car4 = Vehicle('car4', model='miramar', color='Black')
        scenario.add_vehicle(car4, pos=(-708.58203, 115.326, 118.60),
                             rot_quat=(0.0181, -0.00645, 0.3818, 0.9240))

        scenario.make(beamng)

        bng.load_scenario(scenario)
        bng.step(120)
        time.sleep(5)

        ego.poll_sensors()

        classes = bng.get_annotation_classes(bng.get_annotations())
        annotation = ego.sensors['camera'].data['annotation']
        instance = ego.sensors['camera'].data['instance']
        bboxes = Camera.extract_bboxes(annotation, instance, classes)
        bboxes = [b for b in bboxes if b['class'] == 'CAR']
        assert len(bboxes) == 5


def test_noise(beamng):
    with beamng as bng:
        scenario = Scenario('west_coast_usa', 'camera_test')
        vehicle = Vehicle('test_car', model='etk800')

        pos = (-0.3, 1, 1.0)
        direction = (0, 1, 0)
        fov = 120
        resolution = (64, 64)

        cam = Camera(pos, direction, fov, resolution, colour=True, depth=True)
        noise_cam = RandomImageNoise(cam)
        vehicle.attach_sensor('noise_cam', noise_cam)

        lidar = Lidar()
        noise_lidar = RandomLIDARNoise(lidar, mean=0, var=.5)
        vehicle.attach_sensor('noise_lidar', noise_lidar)

        scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675),
                             rot=(0, 0, 45))
        scenario.make(beamng)

        bng.load_scenario(scenario)
        bng.step(120)
        time.sleep(20)
        vehicle.poll_sensors()

        reference_img = np.array(noise_cam._sensor.data['colour'])
        noise_img = np.array(noise_cam.data['colour'])
        assert(not(np.array_equal(noise_img, reference_img)))

        ref_pc = lidar.data['points']
        noise_pc = noise_lidar.data['points']
        assert(not(np.array_equal(ref_pc, noise_pc)))


@pytest.mark.parametrize('shmem', SHMEM_OPTIONS, ids=SHMEM_IDS)
def test_lidar(beamng, shmem):
    with beamng as bng:
        scenario = Scenario('west_coast_usa', 'lidar_test')
        vehicle = Vehicle('test_car', model='etk800')

        lidar = Lidar(shmem=shmem)
        vehicle.attach_sensor('lidar', lidar)

        scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675),
                             rot=(0, 0, 45))
        scenario.make(beamng)

        bng.load_scenario(scenario)
        bng.step(120)
        time.sleep(20)

        vehicle.poll_sensors()

        arr = lidar.data['points']
        ref = arr[0]
        eq = arr[np.where(arr == ref)]
        assert eq.size != arr.size


def test_gforces(beamng):
    with beamng as bng:
        scenario = Scenario('west_coast_usa', 'gforce_test')
        vehicle = Vehicle('test_car', model='etk800')

        gforces = GForces()
        vehicle.attach_sensor('gforces', gforces)

        scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675),
                             rot=(0, 0, 45))
        scenario.make(beamng)

        gx = []
        gy = []
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
    with beamng as bng:
        scenario = Scenario('smallgrid', 'electrics_test')
        vehicle = Vehicle('test_car', model='etk800')

        electrics = Electrics()
        vehicle.attach_sensor('electrics', electrics)

        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(beamng)

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
    with beamng as bng:
        scenario = Scenario('smallgrid', 'damage_test')
        dummy = Vehicle('dummy', model='pickup')
        scenario.add_vehicle(dummy, pos=(0, 0, 0))
        scenario.make(beamng)

        vehicle = Vehicle('test_car', model='etk800')
        damage = Damage()
        vehicle.attach_sensor('damage', damage)

        bng.load_scenario(scenario)
        bng.start_scenario()

        scenario.add_vehicle(vehicle, pos=(0, 0, 32), rot=(-90, 0, 0),
                             cling=False)

        bng.step(600)

        vehicle.poll_sensors()

    assert damage.data['damage'] > 100


def test_state(beamng):
    with beamng as bng:
        scenario = Scenario('smallgrid', 'vehicle_state_test')
        vehicle = Vehicle('test_car', model='pickup')

        state = State()
        vehicle.attach_sensor('newstate', state)

        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(beamng)

        bng.load_scenario(scenario)
        bng.start_scenario()
        bng.step(20)

        vehicle.poll_sensors()

    assert state.data['pos'][0] < 0.1


def test_imu(beamng):
    with beamng as bng:
        scenario = Scenario('smallgrid', 'vehicle_state_test')
        vehicle = Vehicle('test_car', model='etk800')

        imu_pos = IMU(pos=(0.73, 0.51, 0.8), debug=True)
        imu_node = IMU(node=0, debug=True)
        vehicle.attach_sensor('imu_pos', imu_pos)
        vehicle.attach_sensor('imu_node', imu_node)

        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(beamng)

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


def test_ultrasonic(beamng):
    with beamng as bng:
        scenario = Scenario('smallgrid', 'ultrasonic_test')
        cube_dist = 4
        cube = ProceduralCube(name='cube',
                              pos=(0, -cube_dist, 5),
                              rot=None,
                              rot_quat=(0, 0, 0, 1),
                              size=(1, 20, 10))
        scenario.add_procedural_mesh(cube)

        pos = (0, 1, 2)
        rot = (0, 1, 0)
        ultrasonic = Ultrasonic(pos, rot)

        vehicle = Vehicle('test', model='pickup')
        vehicle.attach_sensor('ultrasonic', ultrasonic)

        scenario.add_vehicle(vehicle,
                             pos=(0, 0, 0),
                             rot_quat=(0, 0, 0, 1))
        scenario.make(beamng)

        bng.load_scenario(scenario)
        bng.start_scenario()
        vehicle.poll_sensors()
        assert 0 < vehicle.sensors['ultrasonic'].data['distance'] < cube_dist


if __name__ == '__main__':
    bng = BeamNGpy('localhost', 64256)
    test_ultrasonic(bng)
