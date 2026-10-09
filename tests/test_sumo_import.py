from __future__ import annotations

from pathlib import Path
from typing import Iterator

import pytest

from beamngpy import BeamNGpy, Vehicle
from beamngpy.logging import BNGError
from beamngpy.tools import SumoImporter

from tests.helpers import loaded_scenario

ROAD_DATA = Path(__file__).resolve().parent.parent / "examples" / "data"
SUMO_NODE_FILE = ROAD_DATA / "wcusa.nod.xml"
SUMO_EDGE_FILE = ROAD_DATA / "wcusa.edg.xml"


@pytest.fixture(scope="module")
def import_scene(beamng: BeamNGpy) -> Iterator[BeamNGpy]:
    vehicle = Vehicle("ego", model="etk800")
    with loaded_scenario(
        beamng, level="tech_ground", name="sumo_import", label="SUMO import",
        vehicles=[(vehicle, {"pos": (0, 0, 0), "safe_spawn": True})],
        precompile_shaders=False,
    ) as (bng, _scenario):
        yield bng


def test_sumo_import_imports_roads(import_scene: BeamNGpy):
    assert SUMO_NODE_FILE.is_file() and SUMO_EDGE_FILE.is_file(), (
        f"missing SUMO example pair: {SUMO_NODE_FILE} / {SUMO_EDGE_FILE}"
    )

    result = SumoImporter(import_scene).import_sumo(SUMO_NODE_FILE)

    assert result["importedRoadCount"] > 0, f"no roads imported: {result}"
    assert result["finalized"], f"Road Architect finalize did not run: {result}"
    print(
        f"PASS: imported {result['importedRoadCount']} road(s), "
        f"skipped {result['skippedRoadCount']}"
    )
    for warning in result["warnings"]:
        print(f"Warning: {warning}")


def test_sumo_import_surfaces_engine_error(import_scene: BeamNGpy, tmp_path: Path):
    incomplete = tmp_path / "not_sumo.nod.xml"
    incomplete.write_text("<notSumo />", encoding="utf-8")

    with pytest.raises(BNGError):
        SumoImporter(import_scene).import_sumo(incomplete)