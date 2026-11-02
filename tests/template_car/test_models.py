"""Template car data models: defaults, validation and the optimization variable plumbing."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError

import beamngpy.tools
import beamngpy.tools.template_car as template_car
from beamngpy.tools.template_car import (
    BodyShape,
    Optimization,
    OptimizationEnabled,
    OptimizationVariables,
    Settings,
    SuspensionFront,
    SuspensionRear,
    TargetValues,
    TemplateVehicle,
    VehicleParameters,
    VehicleStructure,
)
from beamngpy.tools.template_car.models import OutputValues
from beamngpy.tools.template_car.utils import jbeam_template_dir

TEMPLATE_DIR = Path(jbeam_template_dir())
TARGET_NAMES = list(OptimizationEnabled.model_fields)

# Documented defaults (UI and BeamNG.tech documentation).
PARAMETER_DEFAULTS = {
    "wheelbase": 2.52,
    "track_width_front": 1.40,
    "track_width_rear": 1.40,
    "front_overhang": 0.68,
    "rear_overhang": 0.70,
    "body_width": 1.60,
    "body_height": 1.32,
    "ride_height": 0.105,
    "front_spring_rate": 27465,
    "front_damping": 5708,
    "rear_spring_rate": 33609,
    "rear_damping": 6362,
    "front_swaybar_spring_rate": 1200,
    "rear_swaybar_spring_rate": 500,
}
VARIABLE_DEFAULTS = {
    "mass_factor": 1.0,
    "cg_y_factor": 0.0,
    "cg_z_factor": 0.0,
    "inertia_yaw_factor": 0.0,
    "inertia_pitch_factor": 0.0,
    "inertia_roll_factor": 0.0,
}
TARGET_DEFAULTS = {
    "mass": 1500,
    "cg_y": 1.3,
    "cg_z": 0.25,
    "inertia_yaw": 1500,
    "inertia_pitch": 1500,
    "inertia_roll": 400,
}

ENABLED_SUBSETS = [
    ("mass",),
    ("cg_z",),
    ("mass", "cg_y", "cg_z"),
    ("inertia_yaw", "inertia_roll"),
    tuple(TARGET_NAMES),
]


def _bounds(model: type, field: str) -> tuple[float, float]:
    metadata = model.model_fields[field].metadata
    lower = next(c.ge for c in metadata if hasattr(c, "ge"))
    upper = next(c.le for c in metadata if hasattr(c, "le"))
    return lower, upper


def _all_enabled() -> OptimizationEnabled:
    return OptimizationEnabled(**{name: True for name in TARGET_NAMES})


def test_vehicle_defaults():
    vehicle = TemplateVehicle(name="Default car")
    assert vehicle.structure == VehicleStructure(
        body_shape=BodyShape.SEDAN,
        suspension_front=SuspensionFront.DOUBLEWISHBONE,
        suspension_rear=SuspensionRear.DOUBLEWISHBONE,
    )
    assert vehicle.parameters.model_dump() == PARAMETER_DEFAULTS
    assert vehicle.optimization.variables.model_dump() == VARIABLE_DEFAULTS
    assert vehicle.optimization.targets.model_dump() == TARGET_DEFAULTS
    assert not vehicle.optimization.required()


def test_vehicle_requires_name():
    with pytest.raises(ValidationError):
        TemplateVehicle()


@pytest.mark.parametrize("field", list(VehicleParameters.model_fields))
def test_parameter_bounds(field):
    lower, upper = _bounds(VehicleParameters, field)
    assert getattr(VehicleParameters(**{field: lower}), field) == lower
    assert getattr(VehicleParameters(**{field: upper}), field) == upper
    margin = (upper - lower) * 1e-3
    for value in (lower - margin, upper + margin):
        with pytest.raises(ValidationError):
            VehicleParameters(**{field: value})


@pytest.mark.parametrize("field", list(TargetValues.model_fields))
def test_target_bounds(field):
    lower, upper = _bounds(TargetValues, field)
    TargetValues(**{field: lower})
    TargetValues(**{field: upper})
    with pytest.raises(ValidationError):
        TargetValues(**{field: upper * 1.01})


def test_structure_accepts_enum_values():
    structure = VehicleStructure(body_shape="wagon", suspension_front="strut", suspension_rear="torsionbeam")
    assert structure.body_shape is BodyShape.WAGON
    assert structure.suspension_front is SuspensionFront.STRUT
    assert structure.suspension_rear is SuspensionRear.TORSIONBEAM


@pytest.mark.parametrize("field", list(VehicleStructure.model_fields))
def test_structure_rejects_unknown_value(field):
    with pytest.raises(ValidationError):
        VehicleStructure(**{field: "hovercraft"})


@pytest.mark.parametrize("shape", list(BodyShape), ids=lambda s: s.value)
def test_body_shape_has_template_assets(shape):
    folder = TEMPLATE_DIR / "body" / shape.value
    for name in (
        f"template_car_body_{shape.value}.jbeam",
        f"template_car_{shape.value}_middle.dae",
        f"template_car_{shape.value}_rear.dae",
        "default.jpg",
    ):
        assert (folder / name).is_file(), name


@pytest.mark.parametrize("suspension", list(SuspensionFront), ids=lambda s: s.value)
def test_front_suspension_has_template(suspension):
    assert (TEMPLATE_DIR / "suspension" / "front" / f"template_car_{suspension.value}_F.jbeam").is_file()


@pytest.mark.parametrize("suspension", list(SuspensionRear), ids=lambda s: s.value)
def test_rear_suspension_has_template(suspension):
    assert (TEMPLATE_DIR / "suspension" / "rear" / f"template_car_{suspension.value}_R.jbeam").is_file()


def test_optimization_not_required_by_default():
    assert not Optimization().required()


@pytest.mark.parametrize("name", TARGET_NAMES)
def test_optimization_required_with_one_target(name):
    assert Optimization(enabled=OptimizationEnabled(**{name: True})).required()


@pytest.mark.parametrize("names", ENABLED_SUBSETS, ids="+".join)
def test_numpy_round_trip(names):
    enabled = OptimizationEnabled(**{name: True for name in names})
    variables = OptimizationVariables(
        mass_factor=1.2,
        cg_y_factor=0.1,
        cg_z_factor=-0.2,
        inertia_yaw_factor=0.3,
        inertia_pitch_factor=-0.4,
        inertia_roll_factor=0.5,
    )
    in_field_order = [name for name in TARGET_NAMES if name in names]

    x = variables.to_numpy(enabled)
    np.testing.assert_allclose(x, [getattr(variables, f"{name}_factor") for name in in_field_order])

    updated = variables.from_numpy(x + 0.01, enabled)
    for name in TARGET_NAMES:
        expected = getattr(variables, f"{name}_factor") + (0.01 if name in names else 0.0)
        assert getattr(updated, f"{name}_factor") == pytest.approx(expected)


def test_variable_metadata_arrays():
    enabled = _all_enabled()
    variables = OptimizationVariables()
    np.testing.assert_array_equal(variables.min_values(enabled), [0.0, -10.0, -10.0, -10.0, -10.0, -10.0])
    np.testing.assert_array_equal(variables.max_values(enabled), [10.0] * 6)
    np.testing.assert_array_equal(variables.min_step(enabled), [1e-4] + [1e-3] * 5)
    np.testing.assert_array_equal(TargetValues().tolerance(enabled), [0.01] + [1e-3] * 5)


def test_metadata_arrays_follow_enabled_subset():
    enabled = OptimizationEnabled(cg_z=True, inertia_roll=True)
    assert OptimizationVariables().min_values(enabled).shape == (2,)
    np.testing.assert_array_equal(TargetValues(cg_z=0.4, inertia_roll=500).to_numpy(enabled), [0.4, 500])


def test_target_names_line_up():
    names = set(TARGET_NAMES)
    assert {name.removesuffix("_factor") for name in OptimizationVariables.model_fields} == names
    assert set(TargetValues.model_fields) == names
    assert set(OutputValues.model_fields) == names


def test_vehicle_json_round_trip():
    vehicle = TemplateVehicle(
        name="Round trip",
        structure=VehicleStructure(body_shape="pickup", suspension_front="liveaxle_4link", suspension_rear="multilink"),
        parameters=VehicleParameters(wheelbase=3.1, ride_height=0.2),
        optimization=Optimization(
            variables=OptimizationVariables(mass_factor=1.4),
            targets=TargetValues(mass=1800, cg_z=0.4),
            enabled=OptimizationEnabled(mass=True, cg_z=True),
        ),
    )
    # Same serialization as save_vehicle_file().
    data = json.loads(json.dumps(vehicle.model_dump()))
    assert TemplateVehicle.model_validate(data) == vehicle


def test_json_schemas():
    vehicle_schema = TemplateVehicle.model_json_schema()
    assert vehicle_schema["required"] == ["name"]
    assert set(vehicle_schema["properties"]) == {"name", "structure", "parameters", "optimization"}
    assert set(Settings.model_json_schema()["properties"]) == {
        "user_folder",
        "vehicles_folder",
        "beamng_install_path",
    }


def test_to_inputs():
    vehicle = TemplateVehicle(name="Inputs car", parameters=VehicleParameters(wheelbase=2.8))
    variables = OptimizationVariables(mass_factor=1.5)
    inputs = vehicle.to_inputs("inputs_car", variables)
    assert (inputs.id, inputs.name) == ("inputs_car", "Inputs car")
    assert inputs.structure == vehicle.structure
    assert inputs.parameters == vehicle.parameters
    assert inputs.variables == variables


def test_public_exports():
    for name in template_car.__all__:
        assert hasattr(template_car, name), name
    assert beamngpy.tools.TemplateCarGenerator is template_car.TemplateCarGenerator
