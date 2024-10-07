from __future__ import annotations

import math
import xml.etree.ElementTree as ET
import re
from typing import TYPE_CHECKING

from beamngpy import MeshRoad, vec3

if TYPE_CHECKING:
    from beamngpy import Scenario

# User control parameters
# The default road width value (when no width is supplied to a 'way').
DEFAULT_WIDTH = 1.0
# The default elevation value (height above sea level).
DEFAULT_ELEVATION = 0.1
# The depth (from bottom to top) of the generated mesh roads in BeamNG.
DEFAULT_DEPTH = 1.0


class Road:
    """
    A container for storing a road polyline, before rendering in BeamNG.
    """

    def __init__(self, name, nodes):
        self.name = name
        self.nodes = nodes


class Way:
    """
    A container for storing an OpenStreetMap way, with any relevant metadata.
    """

    def __init__(self, nodes, width):
        self.nodes = nodes
        self.width = width


class OpenStreetMapImporter:

    # Extracts the road data from the OpenStreetMap file.
    @staticmethod
    def extract_road_data(filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        minLat, maxLat, minLon, maxLon = 0.0, 0.0, 0.0, 0.0
        nodes = {}
        ways = {}
        width = DEFAULT_WIDTH
        for i in root:
            if i.tag == "bounds":
                minLat, maxLat, minLon, maxLon = (
                    float(i.attrib["minlat"]),
                    float(i.attrib["maxlat"]),
                    float(i.attrib["minlon"]),
                    float(i.attrib["maxlon"]),
                )
            elif i.tag == "node":
                nodes[i.attrib["id"]] = vec3(
                    float(i.attrib["lon"]), float(i.attrib["lat"])
                )
            elif i.tag == "way":
                nd = []
                for j in i:
                    if j.tag == "nd":
                        nd.append(j.attrib["ref"])
                    elif j.tag == "tag" and j.attrib["k"] == "width":
                        try:
                            width = float(j.attrib["v"])
                        except ValueError:
                            temp = re.findall(r"\d+", j.attrib["v"])
                            width = list(map(float, temp))
                ways[int(i.attrib["id"])] = Way(nd, width)
        return nodes, ways, minLat, maxLat, minLon, maxLon

    @staticmethod
    def import_osm(filename, scenario: Scenario):

        # Extract the road data primitives from the OpenStreetMap file.
        print("Extracting road data from file...")
        nodes, ways, minLat, maxLat, minLon, maxLon = (
            OpenStreetMapImporter.extract_road_data(filename)
        )
        print("Primitives to import:  nodes:", len(nodes), "; ways:", len(ways))

        # Convert from lattitude/longitude to metres, with world origin (0, 0) at the (minLon, minLat) position.
        for _, n in nodes.items():
            lat = n.y
            n.y = (n.y - minLat) * 111.32
            n.x = (n.x - minLon) * 40075 * math.cos(lat) / 360
            n.x = n.x * 1000
            n.y = n.y * 1000

        # Create the road polylines from the OpenStreetMap 'ways' data.
        roads = []
        for id, w in ways.items():
            nds = []
            for i in range(len(w.nodes)):
                nd = nodes[w.nodes[i]]
                nds.append([nd.x, nd.y, DEFAULT_ELEVATION, w.width, DEFAULT_DEPTH])
            roads.append(Road("imported_" + str(id), nds))

        # Create the all the roads from the road polyline data.
        print("Loading import in scenario...")
        for r in roads:
            mesh_road = MeshRoad(r.name)
            mesh_road.add_nodes(*r.nodes)
            scenario.add_mesh_road(mesh_road)

        print("Import complete.")
