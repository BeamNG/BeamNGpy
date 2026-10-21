"""Road network importer tests: OpenStreetMap, OpenDRIVE and SUMO.

All formats share one tech_ground scenario on a single simulator instance.
Every test starts from a pristine map: the autouse ``clean_map`` fixture
restarts the scenario, which unloads the runtime state that imports create
(DecalRoads, finalize decal meshes, navgraph edits, terraform) and respawns
the ego vehicle at its start pose.
"""

from __future__ import annotations

import inspect
from collections.abc import Iterator
from math import dist, hypot
from pathlib import Path

import pytest

from beamngpy import BeamNGpy, Vehicle
from beamngpy.logging import BNGError
from beamngpy.tools import OpenDriveImporter, OpenStreetMapImporter, SumoImporter

from tests.helpers import loaded_scenario

ROAD_DATA = Path(__file__).resolve().parent.parent / "examples" / "data"
OSM_FILE = ROAD_DATA / "London.osm"
XODR_FILE = ROAD_DATA / "Ex_LHT-Complex-X-Junction.xodr"
SUMO_NODE_FILE = ROAD_DATA / "London.nod.xml"
SUMO_EDGE_FILE = ROAD_DATA / "London.edg.xml"

# London.osm carries no `ele` attributes, so its nodes import at 0.1 m; a
# custom vertical offset is added on top (editor 'Use Custom Offset').
OSM_BASE_Z = 0.1

# Max distance from an anchor point to the nearest imported road edge.
NEAR_TOLERANCE_M = 2.0

# Ego pose used by the vehicle-anchored imports: away from the (0, 0) spawn so
# the assertions cannot pass on a network placed at the spawn pose.
VEHICLE_POSE_XY = (-900.0, 300.0)

OSM = pytest.param(OpenStreetMapImporter, "import_osm", OSM_FILE, id="osm")
XODR = pytest.param(OpenDriveImporter, "import_xodr", XODR_FILE, id="xodr")
SUMO = pytest.param(SumoImporter, "import_sumo", SUMO_NODE_FILE, id="sumo")
CASES = ("importer", "import_method", "import_file")

# Cases for the tests that do not import a file: engine error and signature.
ERROR_CASES = ["osm", "xodr", "sumo"]
IMP = {
    "osm": (OpenStreetMapImporter, "import_osm"),
    "xodr": (OpenDriveImporter, "import_xodr"),
    "sumo": (SumoImporter, "import_sumo"),
}

INVALID_FILES: dict[type, tuple[str, str]] = {
    OpenStreetMapImporter: ("not_osm.osm", "<notOpenStreetMap />"),
    OpenDriveImporter: ("not_open_drive.xodr", "<notOpenDRIVE />"),
    # The .nod.xml exists but its .edg.xml companion does not.
    SumoImporter: ("not_sumo.nod.xml", "<notSumo />"),
}

SIGNATURE_PARAMS: dict[type, list[str]] = {
    OpenStreetMapImporter: [
        "self", "file_path", "placement_xy", "offset_to_terrain",
        "custom_offset", "terraform", "domain_of_influence", "margin",
    ],
    OpenDriveImporter: [
        "self", "file_path", "placement_xy", "offset_to_terrain",
        "custom_offset", "terraform", "domain_of_influence", "margin",
    ],
    SumoImporter: [
        "self", "file_path_or_prefix", "placement_xy", "offset_to_terrain",
        "custom_offset", "terraform", "domain_of_influence", "margin",
    ],
}


@pytest.fixture(scope="module")
def import_scene(beamng: BeamNGpy) -> Iterator[BeamNGpy]:
    """One tech_ground scenario for every importer test."""
    vehicle = Vehicle("ego", model="etk800")
    with loaded_scenario(
        beamng, level="tech_ground", name="road_importers", label="Road importer tests",
        vehicles=[(vehicle, {"pos": (0, 0, 0), "safe_spawn": True})],
        precompile_shaders=False,
    ) as (bng, _scenario):
        yield bng


@pytest.fixture(autouse=True)
def clean_map(import_scene: BeamNGpy) -> None:
    """Restart the scenario before every test to start from a pristine map."""
    import_scene.scenario.restart()
    import_scene.control.resume()


def do_import(import_scene: BeamNGpy, importer, import_method, import_file, **placement):
    result = getattr(importer(import_scene), import_method)(import_file, **placement)
    assert result["importedRoadCount"] > 0, f"no roads imported: {result}"
    assert result["finalized"], f"Road Architect finalize did not run: {result}"
    print(
        f"PASS: imported {result['importedRoadCount']} road(s), "
        f"skipped {result['skippedRoadCount']}"
    )
    for warning in result["warnings"]:
        print(f"Warning: {warning}")
    return result


def assert_network_near(import_scene: BeamNGpy, x: float, y: float, z: float | None = None):
    """Assert an imported road edge lands within NEAR_TOLERANCE_M of (x, y[, z])."""
    roads = import_scene.scenario.get_road_network(
        include_edges=True, drivable_only=False
    )
    nearest = None
    for road in roads.values():
        for edge in road.get("edges", []):
            middle = edge["middle"]
            if z is None:
                distance = hypot(middle[0] - x, middle[1] - y)
            else:
                distance = dist((middle[0], middle[1], middle[2]), (x, y, z))
            if nearest is None or distance < nearest:
                nearest = distance
    assert nearest is not None, "road network query returned no road edges"
    anchor = (x, y) if z is None else (x, y, z)
    assert nearest <= NEAR_TOLERANCE_M, (
        f"no imported road node within {nearest:.1f} m of the anchor {anchor}: "
        f"the network was not placed there"
    )


def teleport_ego(import_scene: BeamNGpy, xy: tuple[float, float]) -> None:
    vehicle = import_scene.scenario.get_vehicle("ego")
    assert vehicle is not None, "scenario vehicle 'ego' not found"
    vehicle.teleport((xy[0], xy[1], 0.0), reset=False)


@pytest.mark.parametrize(CASES, [OSM, XODR, SUMO])
def test_import_roads(import_scene, importer, import_method, import_file):
    if importer is SumoImporter:
        assert SUMO_NODE_FILE.is_file() and SUMO_EDGE_FILE.is_file(), (
            f"missing SUMO example pair: {SUMO_NODE_FILE} / {SUMO_EDGE_FILE}"
        )
    do_import(import_scene, importer, import_method, import_file)


@pytest.mark.parametrize(CASES, [OSM, XODR, SUMO])
def test_defaults_to_vehicle_placement(import_scene, importer, import_method, import_file):
    """The default import must anchor the network at the player vehicle pose."""
    # After a clean restart the ego sits at its (0, 0) spawn pose, so a road
    # edge there can only come from a vehicle-anchored import.
    do_import(import_scene, importer, import_method, import_file)
    assert_network_near(import_scene, 0.0, 0.0)


@pytest.mark.parametrize(CASES, [XODR, SUMO])
def test_places_custom_xy(import_scene, importer, import_method, import_file):
    """A custom (x, y) placement puts the network at that world position."""
    custom_xy = (300.0, 700.0)
    do_import(import_scene, importer, import_method, import_file, placement_xy=custom_xy)
    assert_network_near(import_scene, *custom_xy)


def test_osm_places_xy_and_z(import_scene):
    """OSM supports a custom placement plus a custom offset, terraformed up.

    The terrain is terraformed to the offset network on purpose: un-terraformed
    DecalRoads hug the terrain at finalize, so the offset would not be
    observable in the rendered network otherwise.
    """
    placement_xy = (-400.0, -400.0)
    custom_offset = 8.0
    expected_z = OSM_BASE_Z + custom_offset
    do_import(
        import_scene, OpenStreetMapImporter, "import_osm", OSM_FILE,
        placement_xy=placement_xy, custom_offset=custom_offset, terraform=True,
    )
    assert_network_near(import_scene, placement_xy[0], placement_xy[1], expected_z)


@pytest.mark.parametrize(CASES, [OSM, XODR, SUMO])
def test_offsets_to_terrain(import_scene, importer, import_method, import_file):
    """offset_to_terrain drapes the roads onto the terrain at the vehicle."""
    teleport_ego(import_scene, VEHICLE_POSE_XY)
    do_import(
        import_scene, importer, import_method, import_file,
        offset_to_terrain=True,
    )
    assert_network_near(import_scene, *VEHICLE_POSE_XY)


@pytest.mark.parametrize(CASES, [OSM, XODR, SUMO])
def test_as_in_file(import_scene, importer, import_method, import_file):
    """'as_in_file' keeps the file's own coordinates."""
    do_import(
        import_scene, importer, import_method, import_file, placement_xy="as_in_file"
    )


@pytest.mark.parametrize(CASES, [OSM, XODR, SUMO])
def test_places_at_camera_position(import_scene, importer, import_method, import_file):
    """'camera_position' anchors the network at the free-camera XY."""
    do_import(
        import_scene, importer, import_method, import_file,
        placement_xy="camera_position",
    )


@pytest.mark.parametrize(CASES, [OSM, XODR, SUMO])
def test_terraform_parameters(import_scene, importer, import_method, import_file):
    """Terraform to import with an explicit domain of influence and margin."""
    do_import(
        import_scene, importer, import_method, import_file,
        terraform=True, domain_of_influence=61.0, margin=2.526,
    )


@pytest.mark.parametrize(CASES, [OSM, XODR, SUMO])
def test_terraform_defaults(import_scene, importer, import_method, import_file):
    """Terraform to import with the default values (30 m / 1.0 m)."""
    do_import(
        import_scene, importer, import_method, import_file, terraform=True
    )


@pytest.mark.parametrize("case", ERROR_CASES)
def test_surfaces_engine_error(import_scene, case, tmp_path):
    importer, import_method = IMP[case]
    invalid_name, invalid_text = INVALID_FILES[importer]
    invalid = tmp_path / invalid_name
    invalid.write_text(invalid_text, encoding="utf-8")

    with pytest.raises(BNGError):
        getattr(importer(import_scene), import_method)(invalid)


@pytest.mark.parametrize("case", ERROR_CASES)
def test_rejects_conflicting_z_options(import_scene, case):
    """Only one of 'Offset To Terrain' and 'Use Custom Offset' can be set."""
    importer, import_method = IMP[case]
    with pytest.raises(ValueError):
        getattr(importer(import_scene), import_method)(
            "whatever", offset_to_terrain=True, custom_offset=2.0
        )


@pytest.mark.parametrize("case", ERROR_CASES)
def test_rejects_unknown_options(case):
    """Only the documented file path, placement and terraform fields exist."""
    importer, import_method = IMP[case]
    params = list(inspect.signature(getattr(importer, import_method)).parameters)
    assert params == SIGNATURE_PARAMS[importer], params


# Editor import window defaults (roadArchitect.lua / roadNetworkExporter.lua):
# DOI IntPtr(30), margin FloatPtr(1.0), every checkbox off.
EDITOR_DEFAULTS = {
    "placement_xy": None,
    "offset_to_terrain": False,
    "custom_offset": None,
    "terraform": False,
    "domain_of_influence": 30.0,
    "margin": 1.0,
}


@pytest.mark.parametrize("case", ERROR_CASES)
def test_default_options_match_editor(case):
    """The keyword defaults are the editor 'Import Options' defaults."""
    importer, import_method = IMP[case]
    params = inspect.signature(getattr(importer, import_method)).parameters
    for name, default in EDITOR_DEFAULTS.items():
        assert params[name].default == default, (name, params[name].default)