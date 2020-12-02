import numpy as np
import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.beamngcommon import BNGValueError, BNGError
from beamngpy.sensors import Camera, Lidar, Damage, Electrics, GForces, State
from beamngpy.noise import WhiteGaussianRGBNoise


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

    reference_camera = Camera(pos, direction, fov, resolution, colour=True)
    vehicle.attach_sensor('reference_camera', reference_camera)

    noise_camera = Camera(pos, direction, fov, resolution, colour=True)
    noise_camera = WhiteGaussianRGBNoise(noise_camera, .5, 0)
    vehicle.attach_sensor('noise_cam', noise_camera)

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=(0, 0, 45))
    scenario.make(beamng)

    with beamng as bng:
        bng.load_scenario(scenario)
        bng.step(120)
        vehicle.poll_sensors()

        noise_img = np.asarray(noise_camera.data['colour'])
        reference_img = np.asarray(reference_camera.data['colour'])
        # since this is based on two different renders this will always be different
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

        sensors = bng.poll_sensors(vehicle)

        arr = sensors['lidar']['points']
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
            sensors = bng.poll_sensors(vehicle)
            gx.append(sensors['gforces']['gx'])
            gy.append(sensors['gforces']['gy'])

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

        sensors = bng.poll_sensors(vehicle)

    assert sensors['electrics']['airspeed'] > 0
    assert sensors['electrics']['wheelspeed'] > 0
    assert sensors['electrics']['throttle'] > 0


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

        sensors = bng.poll_sensors(vehicle)

    assert sensors['damage']['damage'] > 100

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

        sensors = bng.poll_sensors(vehicle)

        assert sensors["state"] != None
        assert vehicle.state == sensors["state"]

if __name__=="__main__":
    bng = BeamNGpy('localhost', 64256)
    # test_state(bng)
    test_camera(bng)
