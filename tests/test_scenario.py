from __future__ import annotations

import socket
from typing import TYPE_CHECKING

import pytest

from beamngpy import BeamNGpy, Level, Scenario, Vehicle
from beamngpy.logging import BNGValueError
from beamngpy.sensors import Electrics

if TYPE_CHECKING:
    from beamngpy.scenario.scenario_object import SceneObject


def test_new_scenario(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario('smallgrid', 'test_scenario')
        vehicle = Vehicle('test_car', model='etk800')
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot_quat=(0, 0, 0, 1))
        scenario.make(bng)
        bng.scenario.load(scenario)
        assert bng.scenario.get_name() == 'test_scenario'
        try:
            bng.scenario.start()
        except socket.timeout:
            assert False

        scenario.delete(beamng)

        with pytest.raises(BNGValueError):
            bng.scenario.load(scenario)


def test_no_scenario(beamng: BeamNGpy):
    with beamng as bng:
        bng.control.return_to_main_menu()  # if a scenario was running previously
        with pytest.raises(BNGValueError):
            bng.scenario.get_current()


def test_find_scenario(beamng: BeamNGpy):
    with beamng as bng:
        scenarios = bng.scenario.get_scenarios()
        target = None
        for scenario in scenarios.values():
            if scenario.name == 'scenarios.west_coast_usa.derby_asphalt.title':
                target = scenario
                break

        assert target is not None

        bng.scenario.load(target)

        loaded = bng.scenario.get_current()
        assert loaded.path == target.path


def test_scenario_vehicle_name():
    scenario = Scenario('smallgrid', 'same')
    vehicle = Vehicle('same', model='etk800')
    with pytest.raises(BNGValueError):
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot_quat=(0, 0, 0, 1))


def test_get_scenarios(beamng: BeamNGpy):
    with beamng as bng:
        scenarios = bng.scenario.get_scenarios()
        assert len(scenarios) > 0

        gridmap_scenarios = bng.scenario.get_level_scenarios('gridmap_v2')
        assert len(gridmap_scenarios.values()) > 0

        for scenario in gridmap_scenarios.values():
            assert scenario.level is not None
            assert isinstance(scenario.level, str)
            assert scenario.level == 'gridmap_v2'

        levels = bng.scenario.get_levels()
        gridmap = levels['gridmap_v2']

        ref = gridmap_scenarios
        gridmap_scenarios = bng.scenario.get_level_scenarios(gridmap)

        assert len(gridmap_scenarios.values()) == len(ref.values())

        for scenario in gridmap_scenarios.values():
            assert scenario.level is not None
            assert isinstance(scenario.level, Level)
            assert scenario.level == gridmap


def test_get_level_and_scenarios(beamng: BeamNGpy):
    with beamng as bng:
        levels, scenarios = bng.scenario.get_levels_and_scenarios()
        assert len(levels) > 0
        assert len(scenarios) > 0
        for scenario in scenarios.values():
            scenario_level = scenario.level
            assert isinstance(scenario_level, Level)
            assert scenario.path in scenario_level.scenarios


def test_get_current_vehicles(beamng: BeamNGpy):
    with beamng as bng:
        scenarios = bng.scenario.get_scenarios()
        target = None
        for scenario in scenarios.values():
            if scenario.name == 'scenarios.west_coast_usa.derby_asphalt.title':
                target = scenario
                break

        assert target is not None

        bng.scenario.load(target)

        vehicles = bng.vehicles.get_current()
        player = None
        for vid, vehicle in vehicles.items():
            if vid == 'scenario_player0':
                player = vehicle
                break

        assert player is not None

        sensor = Electrics()
        player.sensors.attach('electrics', sensor)
        player.connect(bng)

        assert player.is_connected()

        bng.scenario.start()

        player.control(throttle=1.0)
        bng.control.step(600)
        player.sensors.poll()
        assert sensor['wheelspeed'] > 0


def find_object_name(scene: SceneObject, name: str):
    if scene.name == name:
        return scene

    for child in scene.children:
        result = find_object_name(child, name)
        if result is not None:
            return result

    return None


def test_get_scenetree(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario('gridmap_v2', 'test_scenario')
        vehicle = Vehicle('egoVehicle', model='etk800')
        scenario.add_vehicle(vehicle, pos=(0, 0, 100), rot_quat=(0, 0, 0, 1))
        scenario.make(bng)
        bng.scenario.load(scenario)
        bng.scenario.start()

        scenario.sync_scene()

        assert scenario.scene is not None
        assert scenario.scene.type == 'SimGroup'

        prefab = find_object_name(scenario.scene, 'test_scenario')
        assert prefab is not None
