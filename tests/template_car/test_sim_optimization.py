"""Target optimization against the real simulator, for the targets exposed in the UI."""

from __future__ import annotations

import io
import json
import re

import numpy as np
import pytest

from beamngpy.tools.template_car import (
    Optimization,
    OptimizationEnabled,
    OptimizationVariables,
    TargetValues,
    TemplateCarGenerator,
    TemplateVehicle,
)
from beamngpy.tools.template_car import optimization
from beamngpy.tools.template_car.models import OutputValues

from .helpers import NUMBER, read_body_nodes, vehicle_files_dir

VEHICLE_ID = "tcg_test_optimize"
# Offsets from the measured default car, so the targets stay reachable on any game version.
TARGET_OFFSETS = {"mass": 150.0, "cg_y": 0.05, "cg_z": 0.02}


def measure(env, vehicle: TemplateVehicle, variables: list[OptimizationVariables]) -> list[OutputValues]:
    """Measure the car like optimize() does: spawn high, then drop the lowest node onto the ground."""
    with optimization.BeamNG(VEHICLE_ID, vehicle, env.user, env.home) as sim:
        z_ref = 10.0
        sim.load_scenario(pos=(0, 0, z_ref), pause=True)
        z_lowest = min(node["pos"][2] for node in sim.v.get_node_info())
        sim.beamng.resume()
        sim.load_scenario(pos=(0, 0, z_ref - z_lowest), pause=True)
        return [
            optimization.measure_output_values(sim.v, vehicle.to_inputs(VEHICLE_ID, var), sim.beamng, env.user)
            for var in variables
        ]


@pytest.fixture(scope="module")
def baseline(optimizer_env) -> OutputValues:
    return measure(optimizer_env, TemplateVehicle(name=VEHICLE_ID), [OptimizationVariables()])[0]


def _vehicle_with_targets(baseline: OutputValues, names) -> TemplateVehicle:
    targets = TargetValues(**{name: getattr(baseline, name) + TARGET_OFFSETS[name] for name in names})
    enabled = OptimizationEnabled(**{name: True for name in names})
    return TemplateVehicle(name=VEHICLE_ID, optimization=Optimization(targets=targets, enabled=enabled))


def _within_tolerance(vehicle: TemplateVehicle, outputs: OutputValues, scale: float = 1.0) -> bool:
    enabled = vehicle.optimization.enabled
    deviations = optimization.deviations(vehicle.optimization.targets, outputs, enabled)
    return bool(np.all(np.abs(deviations) <= scale * vehicle.optimization.targets.tolerance(enabled)))


def test_factors_move_outputs(optimizer_env, baseline):
    heavier, rearward, higher = measure(
        optimizer_env,
        TemplateVehicle(name=VEHICLE_ID),
        [
            OptimizationVariables(mass_factor=1.2),
            OptimizationVariables(cg_y_factor=0.1),
            OptimizationVariables(cg_z_factor=0.2),
        ],
    )
    assert heavier.mass > baseline.mass + 100
    assert rearward.cg_y > baseline.cg_y + 0.01
    assert higher.cg_z > baseline.cg_z + 0.005


@pytest.mark.parametrize("name", list(TARGET_OFFSETS))
def test_optimize_single_target(optimizer_env, baseline, name):
    vehicle = _vehicle_with_targets(baseline, (name,))
    log = io.StringIO()
    var, outputs = optimization.optimize(VEHICLE_ID, vehicle, optimizer_env.user, optimizer_env.home, file=log)

    assert _within_tolerance(vehicle, outputs), log.getvalue()
    defaults = OptimizationVariables()
    assert getattr(var, f"{name}_factor") > getattr(defaults, f"{name}_factor")
    for other in OptimizationEnabled.model_fields:
        if other != name:
            assert getattr(var, f"{other}_factor") == getattr(defaults, f"{other}_factor"), other


def test_optimize_combined_targets(optimizer_env, baseline):
    vehicle = _vehicle_with_targets(baseline, list(TARGET_OFFSETS))
    log = io.StringIO()
    var, outputs = optimization.optimize(VEHICLE_ID, vehicle, optimizer_env.user, optimizer_env.home, file=log)
    assert _within_tolerance(vehicle, outputs), log.getvalue()

    (remeasured,) = measure(optimizer_env, vehicle, [var])
    assert _within_tolerance(vehicle, remeasured, scale=5.0), (outputs, remeasured)


def test_public_api_generate_with_target(optimizer_env, baseline, tmp_path, capsys):
    vehicle_id = "tcg_test_public_api"
    settings_file = tmp_path / "settings.json"
    settings_file.write_text(
        json.dumps({
            "user_folder": optimizer_env.user,
            "vehicles_folder": str(tmp_path / "vehicles"),
            "beamng_install_path": optimizer_env.home,
        }),
        encoding="utf-8",
    )
    generator = TemplateCarGenerator(str(settings_file))
    try:
        generator.generate(vehicle_id, _vehicle_with_targets(baseline, ("mass",)))
        out = capsys.readouterr().out
        assert "Optimization successful." in out
        factor = float(re.search(rf"- Mass: .*factor: ({NUMBER})\)", out)[1])
        assert factor > 1.0

        nodes = read_body_nodes(vehicle_files_dir(optimizer_env.user, vehicle_id), "sedan")
        assert nodes["f2"][3] == pytest.approx(20 * factor)
        assert not vehicle_files_dir(optimizer_env.user, vehicle_id, use_mods_folder=False).exists()
    finally:
        optimization.cleanup(optimizer_env.user, vehicle_id)
