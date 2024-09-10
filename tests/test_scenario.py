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
        scenario = Scenario("tech_ground", "test_scenario")
        vehicle = Vehicle("test_car", model="etk800")
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot_quat=(0, 0, 0, 1))
        scenario.make(bng)
        bng.scenario.load(scenario)
        assert bng.scenario.get_name() == "test_scenario"
        try:
            bng.scenario.start()
        except socket.timeout:
            assert False

        scenario.delete(beamng)

        with pytest.raises(BNGValueError):
            bng.scenario.load(scenario)


def test_no_scenario(beamng: BeamNGpy):
    target = ""
    with beamng as bng:
        bng.control.return_to_main_menu()  # if a scenario was running previously
        with pytest.raises(BNGValueError):
            target = bng.scenario.get_current()
            assert target is None

@pytest.mark.parametrize(
    "scenario_path",
    [
        "/levels/west_coast_usa/scenarios/bus/bus_wcusa.json",  # classic scenario
        "/gameplay/missions/west_coast_usa/aiRace/002-highway/info.json",  # mission
    ],
)
def test_find_scenario(beamng: BeamNGpy, scenario_path: str):
    with beamng as bng:
        scenarios = bng.scenario.get_scenarios()
        # Add print statements to check the structure of scenarios
        print("Available levels:", scenarios.keys())

        # Use .get() to avoid KeyErrors
        scenarios_to_search = scenarios.get("west_coast_usa", []) + scenarios.get("italy", [])
        target = None

        for scenario in scenarios_to_search:
            print(f"Checking scenario: {scenario.path}")
            if scenario.path == scenario_path:
                target = scenario
                break

        assert target is not None, f"Scenario with path {scenario_path} not found."

        bng.scenario.load(
            target, connect_player_vehicle=False, connect_existing_vehicles=False
        )

        loaded = bng.scenario.get_current(connect=False)
        assert loaded.path == target.path


def test_scenario_vehicle_name():
    scenario = Scenario("tech_ground", "same")
    vehicle = Vehicle("same", model="etk800")
    with pytest.raises(BNGValueError):
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot_quat=(0, 0, 0, 1))


def test_get_scenarios(beamng: BeamNGpy):
    with beamng as bng:
        scenarios = bng.scenario.get_scenarios()
        assert len(scenarios) > 0

        gridmap_scenarios = bng.scenario.get_level_scenarios("gridmap_v2")
        assert len(gridmap_scenarios) > 0

        for scenario in gridmap_scenarios:
            assert scenario.level is not None
            assert isinstance(scenario.level, Level)
            assert scenario.level.name == "gridmap_v2"

        levels = bng.scenario.get_levels()
        gridmap = levels["gridmap_v2"]

        ref = gridmap_scenarios
        gridmap_scenarios = bng.scenario.get_level_scenarios(gridmap)

        assert len(gridmap_scenarios) == len(ref)

        for scenario in gridmap_scenarios:
            assert scenario.level is not None
            assert isinstance(scenario.level, Level)
            assert scenario.level == gridmap


def test_get_level_and_scenarios(beamng: BeamNGpy):
    with beamng as bng:
        levels, scenarios = bng.scenario.get_levels_and_scenarios()
        assert len(levels) > 0
        assert len(scenarios) > 0
        for level in scenarios:
            for scenario in scenarios[level]:
                scenario_level = scenario.level
                assert isinstance(scenario_level, Level)
                assert scenario_level.name == level


def test_get_current_vehicles(beamng: BeamNGpy):
    with beamng as bng:
        scenarios = bng.scenario.get_scenarios()
        target = None
        for scenario in scenarios["west_coast_usa"]:
            if scenario.path == "/gameplay/missions/west_coast_usa/aiRace/002-highway/info.json":
                target = scenario
                break

        assert target is not None

        bng.scenario.load(target)

        vehicles = bng.vehicles.get_current(include_config=False)
        player = vehicles["clone"]
        sensor = Electrics()
        player.sensors.attach("electrics", sensor)
        player.connect(bng)

        assert player.is_connected()

        bng.scenario.start()

        player.control(throttle=1.0)
        bng.control.step(600)
        player.sensors.poll()
        assert sensor["wheelspeed"] > 0


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
        scenario = Scenario("gridmap_v2", "test_scenario")
        vehicle = Vehicle("egoVehicle", model="etk800")
        scenario.add_vehicle(vehicle, pos=(0, 0, 100), rot_quat=(0, 0, 0, 1))
        scenario.make(bng)
        bng.scenario.load(scenario)
        bng.scenario.start()

        scenario.sync_scene()

        assert scenario.scene is not None
        assert scenario.scene.type == "SimGroup"

        prefab = find_object_name(scenario.scene, "test_scenario")
        assert prefab is not None
