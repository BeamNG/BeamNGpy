import socket

from beamngpy import BeamNGpy, Scenario, Vehicle, Level, setup_logging
from beamngpy.beamngcommon import BNGValueError
from beamngpy.sensors import Electrics

from time import sleep
import pytest


@pytest.fixture
def beamng():
    beamng = BeamNGpy('localhost', 64256)
    return beamng


def test_new_scenario(beamng):
    with beamng as bng:
        scenario = Scenario('smallgrid', 'test_scenario')
        vehicle = Vehicle('test_car', model='etk800')
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot=(0, 0, 0))
        scenario.make(bng)
        bng.load_scenario(scenario)
        assert bng.get_scenario_name() == 'test_scenario'
        try:
            bng.start_scenario()
            assert True
        except socket.timeout:
            assert False

        scenario.delete(beamng)

        with pytest.raises(BNGValueError):
            bng.load_scenario(scenario)


def test_no_scenario(beamng):
    with beamng as bng:
        with pytest.raises(BNGValueError):
            bng.get_current_scenario()


def test_find_scenario(beamng):
    with beamng as bng:
        scenarios = bng.get_scenarios()
        target = None
        for scenario in scenarios.values():
            if scenario.name == 'scenarios.west_coast_usa.derby_asphalt.title':
                target = scenario
                break

        assert target is not None

        bng.load_scenario(target)

        loaded = bng.get_current_scenario()
        assert loaded.path == scenario.path


def test_scenario_vehicle_name():
    scenario = Scenario('smallgrid', 'same')
    vehicle = Vehicle('same', model='etk800')
    with pytest.raises(BNGValueError):
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot=(0, 0, 0))


def test_get_scenarios(beamng):
    with beamng as bng:
        scenarios = bng.get_scenarios()
        assert len(scenarios) > 0

        gridmap_scenarios = bng.get_level_scenarios('GridMap')
        assert len(gridmap_scenarios.values()) > 0

        for scenario in gridmap_scenarios.values():
            assert scenario.level is not None
            assert isinstance(scenario.level, str)
            assert scenario.level == 'GridMap'

        levels = bng.get_levels()
        gridmap = levels['GridMap']

        ref = gridmap_scenarios
        gridmap_scenarios = bng.get_level_scenarios(gridmap)

        assert len(gridmap_scenarios.values()) == len(ref.values())

        for scenario in gridmap_scenarios.values():
            assert scenario.level is not None
            assert isinstance(scenario.level, Level)
            assert scenario.level == gridmap


def test_get_level_and_scenarios(beamng):
    with beamng as bng:
        levels, scenarios = bng.get_levels_and_scenarios()
        assert len(levels) > 0
        assert len(scenarios) > 0
        for scenario in scenarios.values():
            scenario_level = scenario.level
            assert isinstance(scenario_level, Level)
            assert scenario.path in scenario_level.scenarios


def test_get_current_vehicles(beamng):
    with beamng as bng:
        scenarios = bng.get_scenarios()
        target = None
        for scenario in scenarios.values():
            if scenario.name == 'scenarios.west_coast_usa.derby_asphalt.title':
                target = scenario
                break

        assert target is not None

        bng.load_scenario(target)

        vehicles = bng.get_current_vehicles()
        player = None
        for vid, vehicle in vehicles.items():
            if vid == 'scenario_player0':
                player = vehicle
                break

        assert player is not None

        sensor = Electrics()
        player.attach_sensor('electrics', sensor)
        player.connect(bng)

        assert player.skt is not None

        bng.start_scenario()

        player.control(throttle=1.0)
        bng.step(600)
        player.poll_sensors()
        assert sensor.data['wheelspeed'] > 0


def find_object_name(scene, name):
    if scene.name == name:
        return scene

    for child in scene.children:
        result = find_object_name(child, name)
        if result is not None:
            return result

    return None


def test_get_scenetree(beamng):
    bng = beamng.open()
    # with beamng as bng:
    scenario = Scenario('GridMap', 'test_scenario')
    vehicle = Vehicle('egoVehicle', model='etk800')
    scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot=(0, 0, 0))
    scenario.make(bng)
    bng.load_scenario(scenario)
    bng.start_scenario()

    scenario.sync_scene()

    assert scenario.scene is not None
    assert scenario.scene.type == 'SimGroup'

    prefab = find_object_name(scenario.scene, 'test_scenario')
    assert prefab is not None
