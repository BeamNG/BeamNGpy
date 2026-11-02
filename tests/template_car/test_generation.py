"""Offline generation of the template car files from the Jinja templates."""

from __future__ import annotations

import itertools
import json
import os
import re

import pytest

from beamngpy.tools.template_car import (
    BodyShape,
    OptimizationVariables,
    SuspensionFront,
    SuspensionRear,
    TemplateVehicle,
    VehicleParameters,
    VehicleStructure,
)
from beamngpy.tools.template_car import generation
from beamngpy.tools.template_car.generation import generate_car, node_mass_calculation

from .helpers import NUMBER, read_body_nodes, structure_id, vehicle_files_dir

VEHICLE_ID = "tcg_unit_car"
SHAPES = [shape.value for shape in BodyShape]
ALL_STRUCTURES = [
    VehicleStructure(body_shape=body, suspension_front=front, suspension_rear=rear)
    for body, front, rear in itertools.product(BodyShape, SuspensionFront, SuspensionRear)
]
COLORS = {"sedan": "Orange", "wagon": "Green", "coupe": "Yellow", "pickup": "Blue"}
# Non-trivial factors so that every node gets a distinct weight.
MIXED_VARIABLES = OptimizationVariables(
    mass_factor=1.3,
    cg_y_factor=0.1,
    cg_z_factor=-0.2,
    inertia_yaw_factor=0.03,
    inertia_pitch_factor=-0.02,
    inertia_roll_factor=0.04,
)


def _generate(user_folder, *, structure=None, parameters=None, variables=None, name="Unit car",
              vehicle_id=VEHICLE_ID, use_mods_folder=True):
    vehicle = TemplateVehicle(
        name=name,
        structure=structure or VehicleStructure(),
        parameters=parameters or VehicleParameters(),
    )
    generate_car(vehicle.to_inputs(vehicle_id, variables or OptimizationVariables()), str(user_folder),
                 use_mods_folder=use_mods_folder)
    return vehicle_files_dir(user_folder, vehicle_id, use_mods_folder)


def _node_positions(body_shape: str, parameters: VehicleParameters) -> dict[str, list[float]]:
    return getattr(generation, f"node_positions_{body_shape}")(parameters)


def _expected_files(structure: VehicleStructure) -> set[str]:
    body = structure.body_shape.value
    return {
        "template_car.jbeam",
        "template_car_hubs_F.jbeam",
        "template_car_hubs_R.jbeam",
        "template_car_brakes.jbeam",
        "template_car_engine.jbeam",
        "info.json",
        "info_default.json",
        "default.pc",
        "template_car_front.dae",
        "main.materials.json",
        "default.jpg",
        f"template_car_body_{body}.jbeam",
        f"template_car_{body}_middle.dae",
        f"template_car_{body}_rear.dae",
        f"template_car_{structure.suspension_front.value}_F.jbeam",
        f"template_car_{structure.suspension_rear.value}_R.jbeam",
    }


def _numbers(pattern: str, text: str) -> list[float]:
    return [float(value) for value in re.findall(pattern, text)]


def _node_offset(text: str, slot: str) -> tuple[float, float, float]:
    match = re.search(
        rf'\["{slot}",.*?"nodeOffset":\{{"x":({NUMBER}), "y":({NUMBER}), "z":({NUMBER})\}}', text
    )
    assert match, slot
    return tuple(float(v) for v in match.groups())


def _mirror(node: str) -> str | None:
    if node.endswith("ll"):
        return node[:-2] + "rr"
    if node.endswith("l"):
        return node[:-1] + "r"
    return None


def _mean(nodes, moment) -> float:
    total = sum(m for *_, m in nodes.values())
    return sum(moment(x, y, z) * m for x, y, z, m in nodes.values()) / total


@pytest.mark.parametrize("structure", ALL_STRUCTURES, ids=structure_id)
def test_generate_every_structure(user_folder, structure):
    files_dir = _generate(user_folder, structure=structure)
    body = structure.body_shape.value

    assert {path.name for path in files_dir.iterdir()} == _expected_files(structure)
    for path in files_dir.iterdir():
        if path.suffix in (".jbeam", ".json", ".pc"):
            text = path.read_text(encoding="utf-8")
            assert "{{" not in text and "{%" not in text, path.name

    info = json.loads((files_dir / "info.json").read_text(encoding="utf-8"))
    assert info["Name"] == "Unit car"
    assert info["defaultPaintName1"] == COLORS[body]
    assert COLORS[body] in info["paints"]
    info_default = json.loads((files_dir / "info_default.json").read_text(encoding="utf-8"))
    assert info_default["defaultPaintName1"] == COLORS[body]

    parts = json.loads((files_dir / "default.pc").read_text(encoding="utf-8"))["parts"]
    assert parts["template_car_body"] == f"template_car_body_{body}"
    assert parts["template_car_suspension_F"] == f"template_car_{structure.suspension_front.value}_F"
    assert parts["template_car_suspension_R"] == f"template_car_{structure.suspension_rear.value}_R"

    assert set(read_body_nodes(files_dir, body)) == set(_node_positions(body, VehicleParameters()))


def test_rendered_parameters(user_folder):
    parameters = VehicleParameters(
        wheelbase=2.9,
        track_width_front=1.5,
        track_width_rear=1.7,
        ride_height=0.15,
        front_spring_rate=30001,
        rear_spring_rate=40002,
        front_damping=5001,
        rear_damping=6002,
        front_swaybar_spring_rate=1301,
        rear_swaybar_spring_rate=601,
    )
    files_dir = _generate(user_folder, parameters=parameters)

    main = (files_dir / "template_car.jbeam").read_text(encoding="utf-8")
    assert _node_offset(main, "template_car_suspension_F") == pytest.approx((1.5 / 2 - 0.65, 0.0, 0.15))
    assert _node_offset(main, "template_car_suspension_R") == pytest.approx((1.7 / 2 - 0.60, 2.9, 0.15))
    hubs_rear = (files_dir / "template_car_hubs_R.jbeam").read_text(encoding="utf-8")
    assert _node_offset(hubs_rear, "wheel_R_5") == pytest.approx((1.7 / 2 - 0.48, 2.9, 0.15))

    front = (files_dir / "template_car_doublewishbone_F.jbeam").read_text(encoding="utf-8")
    rear = (files_dir / "template_car_doublewishbone_R.jbeam").read_text(encoding="utf-8")
    for text, own, other in ((front, "front", "rear"), (rear, "rear", "front")):
        springs = _numbers(rf'"beamSpring":({NUMBER})', text)
        dampers = _numbers(rf'"beamDamp":({NUMBER})', text)
        swaybars = _numbers(rf'"spring":({NUMBER})', text)
        assert getattr(parameters, f"{own}_spring_rate") in springs
        assert getattr(parameters, f"{other}_spring_rate") not in springs
        assert getattr(parameters, f"{own}_damping") in dampers
        assert getattr(parameters, f"{other}_damping") not in dampers
        assert getattr(parameters, f"{own}_swaybar_spring_rate") in swaybars
        assert getattr(parameters, f"{other}_swaybar_spring_rate") not in swaybars


@pytest.mark.parametrize("body_shape", SHAPES)
def test_body_geometry(user_folder, body_shape):
    parameters = VehicleParameters(wheelbase=3.0, front_overhang=0.9, rear_overhang=1.1, body_width=1.8, body_height=1.5)
    nodes = read_body_nodes(_generate(user_folder, structure=VehicleStructure(body_shape=body_shape),
                                      parameters=parameters), body_shape)

    for name, position in _node_positions(body_shape, parameters).items():
        assert nodes[name][:3] == pytest.approx(position, abs=5e-4), name
    xs, ys, zs = ([node[i] for node in nodes.values()] for i in range(3))
    assert max(xs) == pytest.approx(0.9) and min(xs) == pytest.approx(-0.9)
    assert min(ys) == pytest.approx(-0.9)
    assert max(ys) == pytest.approx(3.0 + 1.1)
    assert min(zs) == pytest.approx(0.0)
    assert max(zs) == pytest.approx(1.5)


@pytest.mark.parametrize("body_shape", SHAPES)
def test_default_node_weights(user_folder, body_shape):
    nodes = read_body_nodes(_generate(user_folder, structure=VehicleStructure(body_shape=body_shape)), body_shape)
    for name, (*_, weight) in nodes.items():
        assert weight == (10.0 if name.startswith("rf") else 20.0), name


def test_mass_factor_scales_every_node(user_folder, tmp_path):
    base = read_body_nodes(_generate(user_folder), "sedan")
    scaled_folder = tmp_path / "scaled"
    scaled_folder.mkdir()
    scaled = read_body_nodes(_generate(scaled_folder, variables=OptimizationVariables(mass_factor=1.5)), "sedan")
    for name, (*_, weight) in base.items():
        assert scaled[name][3] == pytest.approx(weight * 1.5), name


MOMENTS = {
    "cg_y": lambda x, y, z: y,
    "cg_z": lambda x, y, z: z,
    "inertia_yaw": lambda x, y, z: x * x + y * y,
    "inertia_pitch": lambda x, y, z: x * x + z * z,
    "inertia_roll": lambda x, y, z: y * y + z * z,
}


@pytest.mark.parametrize("target", list(MOMENTS))
def test_factor_moves_mass_distribution(user_folder, tmp_path, target):
    moment = MOMENTS[target]
    means = {}
    for factor in (-0.05, 0.0, 0.05):
        folder = tmp_path / f"user_{factor}"
        folder.mkdir()
        variables = OptimizationVariables(**{f"{target}_factor": factor})
        nodes = read_body_nodes(_generate(folder, variables=variables), "sedan")
        assert all(weight > 0 for *_, weight in nodes.values())
        means[factor] = _mean(nodes, moment)
    assert means[-0.05] < means[0.0] < means[0.05]


@pytest.mark.parametrize("body_shape", SHAPES)
def test_node_weights_follow_mass_model(user_folder, body_shape):
    parameters = VehicleParameters()
    nodes = read_body_nodes(
        _generate(user_folder, structure=VehicleStructure(body_shape=body_shape), variables=MIXED_VARIABLES),
        body_shape,
    )
    v = MIXED_VARIABLES
    for name, (x, y, z) in _node_positions(body_shape, parameters).items():
        base = (10 if name.startswith("rf") else 20) * v.mass_factor
        expected = node_mass_calculation(
            base, x, y, z, v.cg_y_factor, v.cg_z_factor, v.inertia_yaw_factor, v.inertia_pitch_factor,
            v.inertia_roll_factor, parameters.body_height * 0.5,
        )
        assert nodes[name][3] == pytest.approx(expected, rel=1e-12), name


@pytest.mark.parametrize("body_shape", SHAPES)
def test_body_is_mirror_symmetric(user_folder, body_shape):
    nodes = read_body_nodes(
        _generate(user_folder, structure=VehicleStructure(body_shape=body_shape), variables=MIXED_VARIABLES),
        body_shape,
    )
    mirrored = {name: _mirror(name) for name in nodes if _mirror(name)}
    assert mirrored
    for left, right in mirrored.items():
        assert right in nodes, right
        lx, ly, lz, lm = nodes[left]
        rx, ry, rz, rm = nodes[right]
        assert lx > 0 and rx == pytest.approx(-lx), left
        assert (ly, lz) == pytest.approx((ry, rz)), left
        assert lm == pytest.approx(rm, rel=1e-9), left


def test_empty_name_falls_back_to_id(user_folder):
    files_dir = _generate(user_folder, name="")
    assert json.loads((files_dir / "info.json").read_text(encoding="utf-8"))["Name"] == VEHICLE_ID
    assert f'"name":"{VEHICLE_ID}"' in (files_dir / "template_car.jbeam").read_text(encoding="utf-8")


@pytest.mark.parametrize("bad_id", ["", "1car", "Car", "my-car", "my car", "car!", "_car"])
def test_invalid_id(user_folder, bad_id):
    with pytest.raises(ValueError, match="Invalid ID"):
        _generate(user_folder, vehicle_id=bad_id)
    assert not (user_folder / "current").exists()


def test_missing_user_folder(tmp_path):
    missing = tmp_path / "missing"
    with pytest.raises(FileNotFoundError, match="User folder not found"):
        _generate(missing)
    assert not missing.exists()


def test_mods_mode_replaces_previous_output(user_folder):
    files_dir = _generate(user_folder)
    (files_dir / "stale.jbeam").write_text("{}", encoding="utf-8")
    (files_dir.parents[1] / "stale_mod_file.txt").write_text("", encoding="utf-8")

    structure = VehicleStructure(body_shape="wagon", suspension_front="strut", suspension_rear="torsionbeam")
    files_dir = _generate(user_folder, structure=structure)
    assert {path.name for path in files_dir.iterdir()} == _expected_files(structure)
    assert not (files_dir.parents[1] / "stale_mod_file.txt").exists()


def test_vehicles_mode_only_rewrites_changed_files(user_folder):
    files_dir = _generate(user_folder, use_mods_folder=False)
    old = 1_000_000_000
    for path in files_dir.iterdir():
        os.utime(path, (old, old))

    _generate(user_folder, use_mods_folder=False)
    assert all(path.stat().st_mtime == old for path in files_dir.iterdir())

    _generate(user_folder, parameters=VehicleParameters(ride_height=0.2), use_mods_folder=False)
    changed = {path.name for path in files_dir.iterdir() if path.stat().st_mtime != old}
    assert "template_car.jbeam" in changed
    assert "template_car_doublewishbone_F.jbeam" in changed
    assert "default.jpg" not in changed
    assert not any(name.endswith(".dae") for name in changed)


def test_vehicles_mode_keeps_unrelated_files(user_folder):
    files_dir = _generate(user_folder, use_mods_folder=False)
    (files_dir / "notes.txt").write_text("keep", encoding="utf-8")
    _generate(user_folder, use_mods_folder=False)
    assert (files_dir / "notes.txt").read_text(encoding="utf-8") == "keep"


def test_output_is_deterministic(tmp_path):
    outputs = []
    for index, use_mods_folder in enumerate((True, False, True)):
        folder = tmp_path / f"user_{index}"
        folder.mkdir()
        files_dir = _generate(folder, variables=MIXED_VARIABLES, use_mods_folder=use_mods_folder)
        outputs.append({path.name: path.read_bytes() for path in files_dir.iterdir()})
    assert outputs[0] == outputs[1] == outputs[2]
