from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import TYPE_CHECKING

from beamngpy import MeshRoad, vec3

if TYPE_CHECKING:
    from beamngpy import Scenario

# User control parameters
# The default road width value (when no width is supplied to a 'way').
DEFAULT_WIDTH = 1.0
# The default elevation value (height above sea level), if no elevation data supplied.
DEFAULT_ELEVATION = 0.1
# The default number of lanes to assume for a road, if it is not provided.
DEFAULT_NUM_LANES = 2
# The width to use for each lane.
LANE_WIDTH = 1.0
# The depth (from bottom to top) of the generated mesh roads in BeamNG.
DEFAULT_DEPTH = 1.0


class Road:
    """
    A container for storing a road polyline, before rendering in BeamNG.
    """

    def __init__(self, name, nodes):
        self.name = name
        self.nodes = nodes


class Edge:
    """
    A container for storing a Sumo 'edge'.
    """

    def __init__(self, a, b, num_lanes):
        self.a, self.b = a, b
        self.num_lanes = num_lanes


class SumoImporter:

    # Extracts the node data from the Sumo files (.nod.xml).
    @staticmethod
    def extract_node_data(filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        nodes = {}
        for i in root:
            if i.tag == "node":
                if "z" in i.attrib:
                    nodes[i.attrib["id"]] = vec3(
                        float(i.attrib["x"]), float(i.attrib["y"]), float(i.attrib["z"])
                    )
                else:
                    nodes[i.attrib["id"]] = vec3(
                        float(i.attrib["x"]), float(i.attrib["y"]), DEFAULT_ELEVATION
                    )
        return nodes

    # Extracts the edge data from the Sumo files (.edg.xml).
    @staticmethod
    def extract_edge_data(filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        edges = {}
        for i in root:
            if i.tag == "edge":
                if "numLanes" in i.attrib:
                    edges[i.attrib["id"]] = Edge(
                        i.attrib["from"], i.attrib["to"], int(i.attrib["numLanes"])
                    )
                else:
                    edges[i.attrib["id"]] = Edge(
                        i.attrib["from"], i.attrib["to"], DEFAULT_NUM_LANES
                    )
        return edges

    # Remove all duplicate edges (ones where the two nodes are the same, even if in reverse eg A-B and B-A).
    @staticmethod
    def remove_duplicate_edges(edges):
        new_edges = {}
        for k1, v1 in edges.items():
            is_already_in_collection = False
            for _, v2 in new_edges.items():
                if (v1.a == v2.a and v1.b == v2.b) or (v1.a == v2.b and v1.b == v2.a):
                    is_already_in_collection = True
                    break
            if is_already_in_collection == False:
                new_edges[k1] = v1
        return new_edges

    @staticmethod
    def import_sumo(prefix, scenario: Scenario):

        # Extract the road data primitives from the Sumo files.
        print("Extracting road data from Sumo files...")
        nodes = SumoImporter.extract_node_data(prefix + ".nod.xml")
        unprocessed_edges = SumoImporter.extract_edge_data(prefix + ".edg.xml")
        edges = SumoImporter.remove_duplicate_edges(unprocessed_edges)
        print("Primitives to import:  nodes:", len(nodes), "; edges:", len(edges))

        # Create the road polylines from the 'edge' and 'node' data.
        roads = []
        for id, e in edges.items():
            n1, n2 = nodes[e.a], nodes[e.b]
            width = e.num_lanes * LANE_WIDTH
            nds = [
                [n1.x, n1.y, n1.z, width, DEFAULT_DEPTH],
                [n2.x, n2.y, n2.z, width, DEFAULT_DEPTH],
            ]
            roads.append(Road("imported_" + str(id), nds))

        # Create the all the roads from the road polyline data.
        print("Loading import in scenario...")
        for r in roads:
            mesh_road = MeshRoad(r.name)
            mesh_road.add_nodes(*r.nodes)
            scenario.add_mesh_road(mesh_road)

        print("Import complete.")
