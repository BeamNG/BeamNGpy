"""Generated template cars spawned and driven in the simulator."""

from __future__ import annotations

from contextlib import contextmanager
from typing import Iterator

import numpy as np
import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Electrics, PowertrainSensor
from beamngpy.tools.template_car import TemplateVehicle, VehicleParameters

from tests.helpers import loaded_scenario, restart_vehicles, using_sensor, wait_until_poll_valid

from .helpers import STRUCTURE_CASES, generated_car, read_body_nodes, structure_id, vehicle_files_dir

DEFAULT_ID = "tcg_test_default"
SETTLE_STEPS = 300
WHEEL_NODES = ["fw1l", "fw1r", "rw1l", "rw1r"]


@pytest.fixture(scope="module")
def default_car(sim_user) -> Iterator[str]:
    with generated_car(sim_user, DEFAULT_ID):
        yield DEFAULT_ID


@contextmanager
def _car_scenario(
    beamng: BeamNGpy, *, vehicles: list[tuple[Vehicle, dict]], **kwargs
) -> Iterator[tuple[BeamNGpy, Scenario]]:
    """``loaded_scenario`` that despawns its cars on exit, so that ``generated_car`` can delete their files."""
    with loaded_scenario(beamng, vehicles=vehicles, **kwargs) as (bng, scenario):
        try:
            yield bng, scenario
        finally:
            for vehicle, _ in vehicles:
                bng.vehicles.despawn(vehicle)


def _state(vehicle: Vehicle) -> dict:
    vehicle.sensors.poll()
    return vehicle.sensors["state"]


def _assert_settled(bng: BeamNGpy, vehicle: Vehicle) -> None:
    bng.control.step(SETTLE_STEPS, wait=True)
    state = _state(vehicle)
    assert -0.1 < state["pos"][2] < 1.0, state["pos"]
    assert np.linalg.norm(state["vel"]) < 0.2, state["vel"]


def _chosen_parts(vehicle: Vehicle) -> dict[str, str]:
    parts, nodes = {}, [vehicle.get_part_config()["partsTree"]]
    while nodes:
        node = nodes.pop()
        if "id" in node:
            parts[node["id"]] = node.get("chosenPartName")
        # Lua sends an empty children table as a list.
        nodes.extend((node.get("children") or {}).values())
    return parts


def _wheel_geometry(vehicle: Vehicle) -> tuple[float, float, float]:
    """Return (wheelbase, front track, rear track) measured between the wheel hub nodes."""
    fl, fr, rl, rr = (np.array(node["pos"]) for node in vehicle.get_node_info(WHEEL_NODES))
    wheelbase = np.linalg.norm((rl + rr) / 2 - (fl + fr) / 2)
    return wheelbase, np.linalg.norm(fl - fr), np.linalg.norm(rl - rr)


class TestDefaultCar:
    @pytest.fixture(scope="class")
    def scene(self, beamng: BeamNGpy, default_car: str) -> Iterator[tuple[BeamNGpy, Vehicle]]:
        vehicle = Vehicle("tcg_default", model=default_car)
        vehicle.sensors.attach("electrics", Electrics())
        with _car_scenario(
            beamng, name="tcg_default_car", vehicles=[(vehicle, {})], pause=True, precompile_shaders=False,
        ) as (bng, _):
            yield bng, vehicle

    @pytest.fixture(autouse=True)
    def _restart(self, scene):
        restart_vehicles(*scene)

    def test_spawns_and_settles(self, scene):
        _assert_settled(*scene)

    def test_drives(self, scene):
        bng, vehicle = scene
        start = np.array(_state(vehicle)["pos"])
        vehicle.control(throttle=1.0)
        bng.control.step(120, wait=True)
        vehicle.control(throttle=0.0)
        state = _state(vehicle)
        assert np.linalg.norm(np.array(state["pos"]) - start) > 1.0
        assert vehicle.sensors["electrics"]["wheelspeed"] > 0

    def test_powertrain(self, scene):
        bng, vehicle = scene

        def has_motor(readings) -> bool:
            # No samples yet arrive as an empty list, otherwise samples are keyed by index.
            return bool(readings) and any("mainEngine" in sample for sample in readings.values())

        with using_sensor(PowertrainSensor("tcg_powertrain", bng, vehicle, physics_update_time=0.01)) as sensor:
            readings = wait_until_poll_valid(bng, sensor, has_motor, step=30)
        assert has_motor(readings), readings

    def test_node_masses_match_jbeam(self, scene, sim_user, default_car):
        _, vehicle = scene
        expected = read_body_nodes(vehicle_files_dir(sim_user, default_car, use_mods_folder=False), "sedan")
        masses = {node["name"]: node["mass"] for node in vehicle.get_node_info(list(expected))}
        for name, (*_, weight) in expected.items():
            assert masses[name] == pytest.approx(weight, rel=1e-3), name


@pytest.mark.parametrize("index", range(len(STRUCTURE_CASES)), ids=[structure_id(s) for s in STRUCTURE_CASES])
def test_structure_variant(beamng, sim_user, index):
    structure = STRUCTURE_CASES[index]
    vehicle_id = f"tcg_test_variant_{index}"
    with generated_car(sim_user, vehicle_id, TemplateVehicle(name=vehicle_id, structure=structure)):
        vehicle = Vehicle("tcg_variant", model=vehicle_id)
        with _car_scenario(
            beamng, name=f"tcg_variant_{index}", vehicles=[(vehicle, {})], pause=True, precompile_shaders=False,
        ) as (bng, _):
            parts = _chosen_parts(vehicle)
            assert parts["template_car_body"] == f"template_car_body_{structure.body_shape.value}"
            assert parts["template_car_suspension_F"] == f"template_car_{structure.suspension_front.value}_F"
            assert parts["template_car_suspension_R"] == f"template_car_{structure.suspension_rear.value}_R"
            _assert_settled(bng, vehicle)


def test_geometry_follows_parameters(beamng, sim_user, default_car):
    parameters = VehicleParameters(wheelbase=2.82, track_width_front=1.60, track_width_rear=1.50)
    changed_id = "tcg_test_geometry"
    with generated_car(sim_user, changed_id, TemplateVehicle(name=changed_id, parameters=parameters)):
        default = Vehicle("tcg_geometry_default", model=default_car)
        changed = Vehicle("tcg_geometry_changed", model=changed_id)
        with _car_scenario(
            beamng, name="tcg_geometry",
            vehicles=[(default, {"pos": (0, 0, 0)}), (changed, {"pos": (6, 0, 0)})],
            pause=True, precompile_shaders=False,
        ) as (bng, _):
            bng.control.step(SETTLE_STEPS, wait=True)
            delta = np.subtract(_wheel_geometry(changed), _wheel_geometry(default))

    reference = VehicleParameters()
    expected = (
        parameters.wheelbase - reference.wheelbase,
        parameters.track_width_front - reference.track_width_front,
        parameters.track_width_rear - reference.track_width_rear,
    )
    np.testing.assert_allclose(delta, expected, atol=0.02)


@pytest.mark.parametrize("safe_spawn, z_range", [(True, (-0.1, 0.6)), (False, (2.0, 3.5))], ids=["safe", "unsafe"])
def test_safe_spawn(beamng, default_car, safe_spawn, z_range):
    vehicle = Vehicle("tcg_spawn", model=default_car)
    with _car_scenario(
        beamng, level="tech_ground", name=f"tcg_spawn_{safe_spawn}",
        vehicles=[(vehicle, {"pos": (0, 0, 3), "safe_spawn": safe_spawn})],
        pause=True, precompile_shaders=False,
    ) as (bng, _):
        bng.control.step(10, wait=True)
        z = _state(vehicle)["pos"][2]
    assert z_range[0] < z < z_range[1], z
