from __future__ import annotations

from pathlib import Path
from typing import Iterator
from xml.etree import ElementTree

import pytest

import beamngpy.tools.opendrive_export as opendrive_export
import beamngpy.tools.osm_export as osm_export
import beamngpy.tools.sumo_export as sumo_export
from beamngpy import BeamNGpy, Vehicle
from beamngpy.logging import BNGError

from tests.helpers import loaded_scenario


# Road network export tests. A plain filename is resolved by the simulator against its
# (versioned) user folder; absolute host paths outside its mounts are rejected.
EXPORT_LEVEL = "hirochi_raceway"
EXPORT_PREFIX = "road_exporter"
EXPORT_SUFFIXES = (".xodr", ".osm", ".nod.xml", ".edg.xml")


def export_path(bng: BeamNGpy, suffix: str) -> Path:
    """Expected location of an exported file: the simulator's versioned user folder."""
    return Path(bng.user_with_version) / f"{EXPORT_PREFIX}{suffix}"


@pytest.fixture(scope="module")
def export_scene(beamng: BeamNGpy) -> Iterator[BeamNGpy]:
    vehicle = Vehicle("ego", model="etk800")
    with loaded_scenario(
        beamng, level=EXPORT_LEVEL, name="road_export", label="Road network export",
        vehicles=[(vehicle, {"pos": (0, 0, 0), "safe_spawn": True})],
        precompile_shaders=False,
    ) as (bng, _scenario):
        for suffix in EXPORT_SUFFIXES:  # drop stale files from a previous run
            stale = export_path(bng, suffix)
            if stale.exists():
                stale.unlink()
        yield bng


def test_open_drive_export_writes_roads(export_scene: BeamNGpy):
    opendrive_export.OpenDriveExporter.export(EXPORT_PREFIX, export_scene)

    xodr = export_path(export_scene, ".xodr")
    assert xodr.is_file(), f"no .xodr was written to the user folder: {xodr}"

    root = ElementTree.parse(xodr).getroot()
    assert root.tag == "OpenDRIVE"
    assert root.find("header") is not None, "the .xodr file has no header"
    roads = root.findall("road")
    assert len(roads) > 0, "the .xodr file contains no roads"

    for road in roads:
        assert float(road.get("length", 0)) > 0, f"road {road.get('id')} has no length"
        assert road.find("planView/geometry") is not None, (
            f"road {road.get('id')} has no reference-line geometry"
        )
        assert road.find("lanes/laneSection") is not None, (
            f"road {road.get('id')} has no lane section"
        )
    print(f"PASS: exported {len(roads)} road(s) to {xodr}")


def test_open_street_map_export_writes_ways(export_scene: BeamNGpy):
    osm_export.OpenStreetMapExporter.export(EXPORT_PREFIX, export_scene)

    osm = export_path(export_scene, ".osm")
    assert osm.is_file(), f"no .osm was written to the user folder: {osm}"

    root = ElementTree.parse(osm).getroot()
    assert root.tag == "osm" and root.get("version") == "0.6"
    nodes = root.findall("node")
    ways = root.findall("way")
    assert len(nodes) > 0, "the .osm file contains no nodes"
    assert len(ways) > 0, "the .osm file contains no ways"

    node_ids = {node.get("id") for node in nodes}
    for node in nodes:
        float(node.get("lat"))  # raises if lat/lon are not valid coordinates
        float(node.get("lon"))
    for way in ways:
        refs = [nd.get("ref") for nd in way.findall("nd")]
        assert refs, f"way {way.get('id')} has no node references"
        assert all(ref in node_ids for ref in refs), (
            f"way {way.get('id')} references a node that does not exist"
        )
    print(f"PASS: exported {len(nodes)} node(s) and {len(ways)} way(s) to {osm}")


def test_sumo_export_writes_nodes_and_edges(export_scene: BeamNGpy):
    sumo_export.SumoExporter.export(EXPORT_PREFIX, export_scene)

    nod = export_path(export_scene, ".nod.xml")
    edg = export_path(export_scene, ".edg.xml")
    assert nod.is_file(), f"no .nod.xml was written to the user folder: {nod}"
    assert edg.is_file(), f"no .edg.xml was written to the user folder: {edg}"

    nod_root = ElementTree.parse(nod).getroot()
    assert nod_root.tag == "nodes"
    node_ids = {node.get("id") for node in nod_root.findall("node")}
    assert len(node_ids) > 0, "the .nod.xml file contains no junction nodes"

    edg_root = ElementTree.parse(edg).getroot()
    assert edg_root.tag == "edges"
    edges = edg_root.findall("edge")
    assert len(edges) > 0, "the .edg.xml file contains no edges"

    for edge in edges:
        edge_id = edge.get("id")
        assert edge.get("from") in node_ids, f"edge {edge_id} starts at an exported node"
        assert edge.get("to") in node_ids, f"edge {edge_id} ends at an exported node"
        assert int(edge.get("numLanes", 0)) >= 1, f"edge {edge_id} has no lanes"
        assert float(edge.get("speed", 0)) > 0, f"edge {edge_id} has no speed"
    print(f"PASS: exported {len(node_ids)} junction(s) and {len(edges)} edge(s) to {nod.parent}")


def test_export_surfaces_simulator_error(export_scene: BeamNGpy, tmp_path: Path):
    # The simulator sandbox rejects host paths outside its mounts.
    with pytest.raises(BNGError):
        opendrive_export.OpenDriveExporter.export(str(tmp_path / "unwritable"), export_scene)
