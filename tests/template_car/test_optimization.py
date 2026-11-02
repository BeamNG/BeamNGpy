"""Target optimization against a fake simulator with a known, coupled response."""

from __future__ import annotations

import io
from math import tanh
from types import SimpleNamespace

import numpy as np
import pytest

from beamngpy.logging import BNGDisconnectedError
from beamngpy.tools.template_car import (
    Optimization,
    OptimizationEnabled,
    OptimizationVariables,
    TargetValues,
    TemplateVehicle,
)
from beamngpy.tools.template_car import optimization
from beamngpy.tools.template_car.models import OutputValues

from .helpers import read_body_nodes, vehicle_files_dir

TARGET_NAMES = list(OptimizationEnabled.model_fields)
TRUE_FACTORS = dict(zip(TARGET_NAMES, (1.25, 0.4, -0.3, 0.2, -0.15, 0.1)))
TARGET_SUBSETS = [(name,) for name in TARGET_NAMES] + [("mass", "cg_y", "cg_z"), tuple(TARGET_NAMES)]


def model_outputs(var: OptimizationVariables) -> OutputValues:
    """Smooth, saturating and cross-coupled stand-in for the simulator measurement."""
    m, cy, cz = var.mass_factor, var.cg_y_factor, var.cg_z_factor
    return OutputValues(
        mass=1200 * m * (1 + 0.05 * tanh(cy) + 0.05 * tanh(cz)),
        cg_y=1.2 + 0.5 * tanh(cy) + 0.02 * (m - 1),
        cg_z=0.5 + 0.3 * tanh(cz) + 0.01 * tanh(cy),
        inertia_yaw=1500 * m * (1 + 0.5 * tanh(var.inertia_yaw_factor)),
        inertia_pitch=1500 * m * (1 + 0.5 * tanh(var.inertia_pitch_factor)),
        inertia_roll=400 * m * (1 + 0.5 * tanh(var.inertia_roll_factor)),
    )


class FakeOptimizerBeamNG:
    """Replaces ``optimization.BeamNG``; the lowest node is 9.7 m high when spawned at z=10."""

    instances: list["FakeOptimizerBeamNG"] = []

    def __init__(self, id, vehicle, user_folder, install_path):
        self.args = (id, vehicle, user_folder, install_path)
        self.spawns = []
        self.resumes = 0
        self.entered = self.exited = False
        self.v = None
        self.beamng = SimpleNamespace(resume=self._resume)
        FakeOptimizerBeamNG.instances.append(self)

    def _resume(self):
        self.resumes += 1

    def load_scenario(self, pos=(0, 0, 0), pause=False):
        assert pause
        self.spawns.append(pos)
        lowest = pos[2] - 0.3
        self.v = SimpleNamespace(get_node_info=lambda: [{"pos": (0, 0, lowest + 0.5)}, {"pos": (0, 0, lowest)}])

    def __enter__(self):
        self.entered = True
        return self

    def __exit__(self, *exc):
        self.exited = True


@pytest.fixture
def fake_sim(monkeypatch):
    FakeOptimizerBeamNG.instances = []
    evaluations = []

    def measure(v, inputs, bng, user_folder):
        evaluations.append(inputs.variables)
        return model_outputs(inputs.variables)

    monkeypatch.setattr(optimization, "BeamNG", FakeOptimizerBeamNG)
    monkeypatch.setattr(optimization, "measure_output_values", measure)
    return SimpleNamespace(instances=FakeOptimizerBeamNG.instances, evaluations=evaluations)


def _vehicle_with_targets(names) -> tuple[TemplateVehicle, OptimizationVariables]:
    enabled = OptimizationEnabled(**{name: True for name in names})
    solution = OptimizationVariables(**{f"{name}_factor": TRUE_FACTORS[name] for name in names})
    targets = TargetValues(**{name: getattr(model_outputs(solution), name) for name in names})
    vehicle = TemplateVehicle(name="Optimized", optimization=Optimization(targets=targets, enabled=enabled))
    return vehicle, solution


@pytest.mark.parametrize("names", TARGET_SUBSETS, ids="+".join)
def test_optimize_reaches_targets(fake_sim, names):
    vehicle, solution = _vehicle_with_targets(names)
    log = io.StringIO()
    var, outputs = optimization.optimize("opt_car", vehicle, "user", "install", file=log)

    enabled = vehicle.optimization.enabled
    tolerance = vehicle.optimization.targets.tolerance(enabled)
    assert np.all(np.abs(optimization.deviations(vehicle.optimization.targets, outputs, enabled)) <= tolerance)
    assert outputs == model_outputs(var)
    for name in TARGET_NAMES:
        expected = getattr(solution if name in names else vehicle.optimization.variables, f"{name}_factor")
        assert getattr(var, f"{name}_factor") == pytest.approx(expected, abs=5e-3), name
    assert "Optimization successful. Converged after" in log.getvalue()

    (sim,) = fake_sim.instances
    assert sim.args == ("opt_car", vehicle, "user", "install")
    assert sim.entered and sim.exited
    assert sim.spawns[0] == (0, 0, 10.0)
    assert sim.spawns[1][2] == pytest.approx(0.3)
    assert sim.resumes == 1
    for evaluated in fake_sim.evaluations:
        assert 0.0 <= evaluated.mass_factor <= 10.0


def test_optimize_starts_from_current_variables(fake_sim):
    vehicle, _ = _vehicle_with_targets(("mass",))
    vehicle.optimization.variables = OptimizationVariables(mass_factor=2.0, cg_z_factor=0.5)
    var, _ = optimization.optimize("opt_car", vehicle, "user", "install")
    assert fake_sim.evaluations[0].mass_factor == 2.0
    assert var.cg_z_factor == 0.5


def test_unreachable_target_raises(fake_sim):
    vehicle = TemplateVehicle(
        name="Unreachable",
        optimization=Optimization(targets=TargetValues(cg_z=2.0), enabled=OptimizationEnabled(cg_z=True)),
    )
    with pytest.raises(ValueError, match="Optimization failed: Maximum iterations reached without convergence."):
        optimization.optimize("opt_car", vehicle, "user", "install", file=io.StringIO())
    assert fake_sim.instances[0].exited
    assert all(-10.0 <= v.cg_z_factor <= 10.0 for v in fake_sim.evaluations)


def test_generate_optimal_car(fake_sim, user_folder):
    vehicle, solution = _vehicle_with_targets(("mass",))
    log = io.StringIO()
    optimization.generate_optimal_car("opt_car", vehicle, str(user_folder), "install", file=log)

    text = log.getvalue()
    assert text.startswith("Starting optimization using BeamNG. Please wait...\n")
    assert "Optimization successful." in text
    factor = float(text.split("- Mass: ")[1].split("factor: ")[1].split(")")[0])
    assert factor == pytest.approx(solution.mass_factor, abs=1e-3)
    assert text.count("(no target, factor: ") == 5

    nodes = read_body_nodes(vehicle_files_dir(user_folder, "opt_car"), "sedan")
    assert nodes["f2"][3] == pytest.approx(20 * factor)


def test_generate_without_targets_skips_simulator(fake_sim, user_folder):
    log = io.StringIO()
    optimization.generate_optimal_car("plain_car", TemplateVehicle(name="Plain"), str(user_folder), "install", file=log)
    assert log.getvalue() == ""
    assert not fake_sim.instances
    assert vehicle_files_dir(user_folder, "plain_car").is_dir()


def test_deviations():
    targets = TargetValues(mass=1000, cg_z=0.5, inertia_roll=300)
    outputs = OutputValues(mass=1100, cg_y=1, cg_z=0.4, inertia_yaw=1, inertia_pitch=1, inertia_roll=350)
    enabled = OptimizationEnabled(mass=True, cg_z=True, inertia_roll=True)
    np.testing.assert_allclose(optimization.deviations(targets, outputs, enabled), [100, -0.1, 50])
    assert optimization.deviations(targets, outputs, OptimizationEnabled()).shape == (0,)


def test_get_mass_properties():
    sample = {
        "mass_properties": {"mass": 1500, "center_of_gravity": [0.1, 1.5, 0.6], "inertia": {"x": 1, "y": 2, "z": 3}},
        "nodes": [{"pos": [0.7, 0.2, 0.3]}, {"pos": [-0.7, 0.0, 0.3]}],
    }
    p = optimization.get_mass_properties(sample)
    assert p["mass"] == 1500
    assert p["center_of_gravity"] == pytest.approx((0.1, 1.4, 0.3))
    assert p["inertia"] == {"x": 1, "y": 2, "z": 3}


def test_print_result():
    var = OptimizationVariables(mass_factor=1.5, cg_z_factor=0.25)
    outputs = OutputValues(mass=1600, cg_y=1.2, cg_z=0.5, inertia_yaw=1000, inertia_pitch=900, inertia_roll=300)
    targets = TargetValues(mass=1500, cg_z=0.5)
    log = io.StringIO()
    optimization.print_result(var, outputs, targets, OptimizationEnabled(mass=True, cg_z=True), file=log)
    assert log.getvalue().splitlines() == [
        "- Mass: 1600.0 (target: 1500.0, deviation: 100.0, factor: 1.5)",
        "- Center of gravity longitudinal (y): 1.2 (no target, factor: 0.0)",
        "- Center of gravity height (z): 0.5 (target: 0.5, deviation: 0.0, factor: 0.25)",
        "- Yaw Inertia: 1000.0 (no target, factor: 0.0)",
        "- Pitch Inertia: 900.0 (no target, factor: 0.0)",
        "- Roll Inertia: 300.0 (no target, factor: 0.0)",
    ]


class _FakeSim:
    def __init__(self):
        self.calls = []
        self.scenario = SimpleNamespace(restart=lambda: self.calls.append("restart"))

    def pause(self):
        self.calls.append("pause")


def test_measure_output_values_maps_axes(user_folder, monkeypatch):
    sim = _FakeSim()
    sample = {
        "time": 10.0,
        "mass_properties": {"mass": 1400, "center_of_gravity": [0.0, 1.3, 0.55], "inertia": {"x": 1700, "y": 450, "z": 1900}},
        "nodes": [{"pos": [0.7, 0.0, 0.3]}, {"pos": [-0.7, 0.0, 0.3]}],
    }
    monkeypatch.setattr(optimization, "reload_vehicle", lambda v: sim.calls.append("reload"))
    monkeypatch.setattr(optimization, "measure_samples", lambda v, bng: [{"time": 0.0}, sample])
    inputs = TemplateVehicle(name="Measured").to_inputs("measured", OptimizationVariables())

    outputs = optimization.measure_output_values(None, inputs, sim, str(user_folder))
    assert outputs.model_dump() == pytest.approx(
        dict(mass=1400, cg_y=1.3, cg_z=0.25, inertia_yaw=1900, inertia_pitch=1700, inertia_roll=450)
    )
    assert sim.calls == ["pause", "restart", "reload"]
    assert vehicle_files_dir(user_folder, "measured", use_mods_folder=False).is_dir()
    assert not vehicle_files_dir(user_folder, "measured").exists()


def test_measure_output_values_reports_instability(user_folder, monkeypatch):
    def disconnected(v, bng):
        raise BNGDisconnectedError("gone")

    monkeypatch.setattr(optimization, "reload_vehicle", lambda v: None)
    monkeypatch.setattr(optimization, "measure_samples", disconnected)
    inputs = TemplateVehicle(name="Unstable").to_inputs("unstable", OptimizationVariables())
    with pytest.raises(RuntimeError, match="stopped by the simulator"):
        optimization.measure_output_values(None, inputs, _FakeSim(), str(user_folder))


@pytest.fixture
def recorder(monkeypatch):
    state = SimpleNamespace(batches=[], started=None, stopped=False, resumed=False)
    monkeypatch.setattr(optimization, "sleep", lambda s: None)
    monkeypatch.setattr(optimization, "start_recording", lambda v, quantities, steps: setattr(state, "started", (quantities, steps)))
    monkeypatch.setattr(optimization, "fetch_recorded_data", lambda v: state.batches.pop(0) if state.batches else [])
    monkeypatch.setattr(optimization, "stop_recording", lambda v: setattr(state, "stopped", True))
    state.bng = SimpleNamespace(resume=lambda: setattr(state, "resumed", True))
    return state


def test_measure_samples_collects_until_final_time(recorder):
    recorder.batches = [[], [{"time": 4.0}, {"time": 8.0}], [{"time": 10.0}, {"time": 10.5}]]
    data = optimization.measure_samples(None, recorder.bng)
    assert [d["time"] for d in data] == [4.0, 8.0, 10.0]
    quantities, steps = recorder.started
    assert [q["name"] for q in quantities] == ["mass_properties", "node_info"]
    assert quantities[1]["data"] == {"nodes": ["fw1l", "fw1r"]}
    assert steps == 100
    assert recorder.resumed and recorder.stopped


def test_measure_samples_detects_hang(recorder):
    with pytest.raises(RuntimeError, match="hanging for too long"):
        optimization.measure_samples(None, recorder.bng)
    assert not recorder.stopped


class FakeBeamNGpy:
    def __init__(self, host, port, home=None, user=None):
        self.calls = [("init", host, port, home, user)]
        self.settings = SimpleNamespace(change=lambda key, value: self.calls.append(("setting", key, value)))

    def open(self, extensions, *args):
        self.calls.append(("open", extensions, args))

    def set_deterministic(self, speed_factor):
        self.calls.append(("deterministic", speed_factor))

    def close(self):
        self.calls.append(("close",))


def test_context_manager_prepares_and_cleans_up(user_folder, monkeypatch):
    monkeypatch.setattr(optimization, "BeamNGpy", FakeBeamNGpy)
    stale = vehicle_files_dir(user_folder, "ctx_car").parents[1]
    stale.mkdir(parents=True)
    vehicles_dir = vehicle_files_dir(user_folder, "ctx_car", use_mods_folder=False)

    with optimization.BeamNG("ctx_car", TemplateVehicle(name="Ctx"), str(user_folder), "install") as sim:
        assert not stale.exists()
        assert (vehicles_dir / "template_car.jbeam").is_file()
        assert sim.beamng.calls == [
            ("init", "localhost", 25252, "install", str(user_folder)),
            ("open", [], ("-gfx", "vk")),
            ("setting", "fpsLimitEnabled", False),
            ("setting", "fpsLimitBackgroundEnabled", False),
            ("deterministic", 1),
        ]
    assert not vehicles_dir.exists()
    assert sim.beamng.calls[-1] == ("close",)


def test_cleanup(user_folder):
    mods = vehicle_files_dir(user_folder, "gone").parents[1]
    vehicles = vehicle_files_dir(user_folder, "gone", use_mods_folder=False)
    mods.mkdir(parents=True)
    vehicles.mkdir(parents=True)
    optimization.cleanup(str(user_folder), "gone")
    assert not mods.exists() and not vehicles.exists()
    optimization.cleanup(str(user_folder), "gone")
