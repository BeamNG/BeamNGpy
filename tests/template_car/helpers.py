from __future__ import annotations

import re
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

from beamngpy.tools.template_car import TemplateVehicle, VehicleStructure
from beamngpy.tools.template_car import optimization
from beamngpy.tools.template_car.generation import generate_car

NUMBER = r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?"
_BODY_NODE = re.compile(
    rf'\["(\w+)",\s*({NUMBER}),\s*({NUMBER}),\s*({NUMBER}),\s*\{{"nodeWeight":\s*({NUMBER})\}}\]'
)

# Together these cover every body shape, front suspension and rear suspension at least once.
STRUCTURE_CASES = [
    VehicleStructure(body_shape=body, suspension_front=front, suspension_rear=rear)
    for body, front, rear in [
        ("sedan", "doublewishbone", "doublewishbone"),
        ("wagon", "strut", "strut"),
        ("coupe", "liveaxle_4link", "liveaxle_4link"),
        ("pickup", "liveaxle_4link", "liveaxle_4link_dually"),
        ("sedan", "strut", "torsionbeam"),
        ("wagon", "doublewishbone", "semitrailing"),
        ("coupe", "doublewishbone", "multilink"),
    ]
]


def structure_id(structure: VehicleStructure) -> str:
    return "-".join(
        (structure.body_shape.value, structure.suspension_front.value, structure.suspension_rear.value)
    )


def vehicle_files_dir(user_folder: str | Path, vehicle_id: str, use_mods_folder: bool = True) -> Path:
    current = Path(user_folder) / "current"
    if use_mods_folder:
        return current / "mods" / "unpacked" / vehicle_id / "vehicles" / vehicle_id
    return current / "vehicles" / vehicle_id


def read_body_nodes(files_dir: Path, body_shape: str) -> dict[str, tuple[float, float, float, float]]:
    """Return ``{node: (x, y, z, nodeWeight)}`` of the generated body jbeam (JBeam is not strict JSON)."""
    text = (files_dir / f"template_car_body_{body_shape}.jbeam").read_text(encoding="utf-8")
    return {m[1]: tuple(float(v) for v in m.groups()[1:]) for m in _BODY_NODE.finditer(text)}


@contextmanager
def generated_car(
    user_folder: str, vehicle_id: str, vehicle: TemplateVehicle | None = None
) -> Iterator[TemplateVehicle]:
    """Generate a car into ``<user>/current/vehicles`` and remove it afterwards.

    A running BeamNG picks up new files there, while unpacked mods are only mounted at startup.
    """
    if vehicle is None:
        vehicle = TemplateVehicle(name=vehicle_id)
    generate_car(vehicle.to_inputs(vehicle_id, vehicle.optimization.variables), user_folder, use_mods_folder=False)
    try:
        yield vehicle
    finally:
        optimization.cleanup(user_folder, vehicle_id)
