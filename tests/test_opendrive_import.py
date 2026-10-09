from __future__ import annotations

from pathlib import Path
from typing import Iterator

import pytest

from beamngpy import BeamNGpy, Vehicle
from beamngpy.logging import BNGError
from beamngpy.tools import OpenDriveImporter

from tests.helpers import loaded_scenario

ROAD_DATA = Path(__file__).resolve().parent.parent / "examples" / "data"
XODR_FILE = ROAD_DATA / "Ex_LHT-Complex-X-Junction.xodr"


@pytest.fixture(scope="module")
def import_scene(beamng: BeamNGpy) -> Iterator[BeamNGpy]:
    vehicle = Vehicle("ego", model="etk800")
    with loaded_scenario(
        beamng, level="tech_ground", name="opendrive_import", label="OpenDRIVE import",
        vehicles=[(vehicle, {"pos": (0, 0, 0), "safe_spawn": True})],
        precompile_shaders=False,
    ) as (bng, _scenario):
        yield bng


def test_open_drive_import_imports_roads(import_scene: BeamNGpy):
    result = OpenDriveImporter(import_scene).import_xodr(XODR_FILE)

    assert result["importedRoadCount"] > 0, f"no roads imported: {result}"
    assert result["finalized"], f"Road Architect finalize did not run: {result}"
    print(
        f"PASS: imported {result['importedRoadCount']} road(s), "
        f"skipped {result['skippedRoadCount']}"
    )
    for warning in result["warnings"]:
        print(f"Warning: {warning}")


def test_open_drive_import_surfaces_engine_error(import_scene: BeamNGpy, tmp_path: Path):
    invalid = tmp_path / "not_open_drive.xodr"
    invalid.write_text("<notOpenDRIVE />", encoding="utf-8")

    with pytest.raises(BNGError):
        OpenDriveImporter(import_scene).import_xodr(invalid)