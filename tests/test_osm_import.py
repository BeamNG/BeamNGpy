from __future__ import annotations

from pathlib import Path
from typing import Iterator

import pytest

from beamngpy import BeamNGpy, Vehicle
from beamngpy.logging import BNGError
from beamngpy.tools import OpenStreetMapImporter

from tests.helpers import loaded_scenario

ROAD_DATA = Path(__file__).resolve().parent.parent / "examples" / "data"
OSM_FILE = ROAD_DATA / "London.osm"


@pytest.fixture(scope="module")
def import_scene(beamng: BeamNGpy) -> Iterator[BeamNGpy]:
    vehicle = Vehicle("ego", model="etk800")
    with loaded_scenario(
        beamng, level="tech_ground", name="osm_import", label="OpenStreetMap import",
        vehicles=[(vehicle, {"pos": (0, 0, 0), "safe_spawn": True})],
        precompile_shaders=False,
    ) as (bng, _scenario):
        yield bng


def test_osm_import_imports_roads(import_scene: BeamNGpy):
    result = OpenStreetMapImporter(import_scene).import_osm(OSM_FILE)

    assert result["importedRoadCount"] > 0, f"no roads imported: {result}"
    assert result["finalized"], f"Road Architect finalize did not run: {result}"
    print(
        f"PASS: imported {result['importedRoadCount']} road(s), "
        f"skipped {result['skippedRoadCount']}"
    )
    for warning in result["warnings"]:
        print(f"Warning: {warning}")


def test_osm_import_surfaces_engine_error(import_scene: BeamNGpy, tmp_path: Path):
    invalid = tmp_path / "not_osm.osm"
    invalid.write_text("<notOpenStreetMap />", encoding="utf-8")

    with pytest.raises(BNGError):
        OpenStreetMapImporter(import_scene).import_osm(invalid)