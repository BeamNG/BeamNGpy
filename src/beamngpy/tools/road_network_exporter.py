from __future__ import annotations

import math
from datetime import datetime
from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any

import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib import collections as mc

from beamngpy import vec3
from beamngpy.logging import LOGGER_ID, BNGError
from beamngpy.sensors.communication_utils import (send_sensor_request,
                                                  set_sensor)
from beamngpy.types import StrDict

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy

__all__ = ['Road_Network_Exporter']


class explicit_cubic:
    """
    A class for representing explicit cubic polynomials of the form: [ p(x) := a + b*x + c*x^2 + d*x^3 ].
    """

    def __init__(self, a, b, c, d):
        """
        Creates an explicit cubic polynomial

        Args:
            a: The coefficient of the constant term.
            b: The coefficient of the linear term (multiplies x).
            c: The coefficient of the quadratic term (multiplies x^2).
            d: The coefficient of the cubic term (multiplies x^3).
        """
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def eval(self, x) -> float:
        """
        Evaluates this cubic polynomial, at the given value.

        Args:
            x: The value at which to evaluate this cubic polynomial.

        Returns:
            The evaluation of the cubic polynomial, p(x).
        """
        x_sq = x * x
        return self.a + (x * self.b) + (x_sq * self.c) + (x * x_sq * self.d)

    def eval_upper_only(self, x) -> float:
        """
        Evaluates the upper two terms (quadratic and cubic terms only) of this cubic polynomial, at the given value.

        Args:
            x: The value at which to evaluate this cubic polynomial.

        Returns:
            The evaluation of the upper two terms of this cubic polynomial, p(x).
        """
        x_sq = x * x
        return self.a + (x_sq * self.c) + (x * x_sq * self.d)

    def approx_length(self, x0, x1, n=9000) -> float:
        """
        Computes a numerical approximation of the arc-length of this cubic polynomial.
        The polynomial is approximated as a polyline, and the length of each subsection is summed to a total length.

        Args:
            x0: The start value in x.
            x1: The end value in x.
            n (optional): The number of subdivisions to use across the polynomial

        Returns:
            A numerical approximation of the arc-length of this cubic polynomial.
        """
        div = (x1 - x0) / float(n)
        sum = 0.0
        last = [x0, self.eval(x0)]
        # Evaluate the cubic at n points. Store them.
        for i in range(n):
            x = x0 + (i * div)
            y = self.eval(x)
            dx = x - last[0]
            dy = y - last[1]
            sum += math.sqrt((dx * dx) + (dy * dy))
            last = [x, y]
        # Return the L^2 distance (approximation).
        return sum


class parametric_cubic:
    """
    A class for representing parametric cubic polynomials of the form:
    [ u(x) := Bu*x + Cu^2 + Du^3, v(x) := Bv*x + Cv^2 + Dv^3 ].
    """

    def __init__(self, Bu, Cu, Du, Cv, Dv):
        """
        Creates a parametric cubic polynomial.

        Args:
            Bu: The linear term of the equation 'u'.
            Cu: The quadratic term of the equation 'u'.
            Du: The cubic term of the equation 'u'.
            Cv: The quadratic term of the equation 'v'.
            Dv: The cubic term of the equation 'v'.
        """
        self.Bu = Bu
        self.Cu = Cu
        self.Du = Du
        self.Cv = Cv
        self.Dv = Dv


class Road:
    """
    A container for storing single sections of roads, which can be represented by a single geometric primitive.
    """

    def __init__(self, start, end, p1, length, hdg, start_elevation, linear_elevation, start_width, linear_width, cubic,
                 predecessor=None, successor=None, junction=None, contact_point=None):
        self.start = start
        self.end = end
        self.p1 = p1
        self.hdg = hdg
        self.start_elevation = start_elevation
        self.linear_elevation = linear_elevation
        self.start_width = start_width
        self.linear_width = linear_width
        self.length = length
        self.Bu = cubic.Bu
        self.Cu = cubic.Cu
        self.Du = cubic.Du
        self.Cv = cubic.Cv
        self.Dv = cubic.Dv
        self.predecessor = predecessor
        self.successor = successor
        self.junction = junction
        self.contact_point = contact_point

    def update_connection_data(self, predecessor, successor, junction, contact_point):
        """
        Updates the road/junction connectivity data associated with this Road.
        """
        self.predecessor = predecessor
        self.successor = successor
        self.junction = junction
        self.contact_point = contact_point


class Junction:
    """
    A class for storing road network junction information.
    """

    def __init__(self, id, connection_roads, is_end_point):
        self.id = id
        self.connection_roads = connection_roads
        self.is_end_point = is_end_point


class Connection:
    """
    A class for storing connectivity information between roads and junctions.
    """

    def __init__(self, connection_type, id, contact_point):
        self.connection_type = connection_type
        self.id = id
        self.contact_point = contact_point


class Road_Network_Exporter:
    """
    A class for retrieving and exporting BeamNG road network data.
    """

    def _to_vec3(self, d):
        """
        Converts a dictionary of 3-element lists to a dictionary of vec3s.
        Format of input: {'a': [1, 2, 3], 'b': [4, 5, 6], ...}

        Args:
            d: The given dictionary containing 3-element lists.

        Returns:
            The dictionary of vec3s.
        """
        cast = {}
        for k, v in d.items():
            cast[k] = vec3(v[0], v[1], v[2])
        return cast

    def _collection_does_not_contain_segment(self, collection, segment) -> bool:
        """
        Determines if a given collection of road segments contains a given road segment.

        Args:
            collection: The given collection of segments.
            segment: The singular given segment, to check the collection with.

        Returns:
            True if the collection contains the given segment, otherwise False.
        """
        for _, v in collection.items():
            matches = 0
            for i in range(len(v)):
                for j in range(len(segment)):
                    if v[i] == segment[j]:
                        matches = matches + 1
            if matches == len(segment):
                return False
        return True

    def compute_path_segments(self) -> dict:
        """
        Populates a dictionary with all individual 'path segments' from the current BeamNG road network.
        Each 'path segment' contains an ordered list of keys to the road graph, with a junction at each end, and continuing road sections in between.

        Returns:
            A dictionary of individual 'path segments' from the loaded road network, indexed by a unique Id number.
        """
        collection = {}
        ctr = 0
        for head_key in self.graph.keys():
            successors = self.graph[head_key].keys()
            if len(successors) != 2:
                for child_key in successors:
                    current_path = []
                    current_path.append(head_key)
                    next_key = child_key
                    while(True):
                        current_path.append(next_key)
                        next_successors = self.graph[next_key].keys()
                        if len(next_successors) != 2:
                            if self._collection_does_not_contain_segment(collection, current_path):
                                collection[ctr] = current_path
                                ctr = ctr + 1
                            break
                        did_find = False
                        for next_successor_key in next_successors:
                            if next_successor_key not in current_path:
                                next_key = next_successor_key
                                did_find = True
                                break
                        if did_find == False:
                            if self._collection_does_not_contain_segment(collection, current_path):
                                collection[ctr] = current_path
                                ctr = ctr + 1
                            break
        return collection

    def _graph_key_to_junction_map(self, path_segments):
        """
        Create a unique map between junction keys (in the road graph data) and unique Id numbers which we attribute to each one found.

        Args:
            path_segments: The collection of path segments, traced from the road graph data.

        Returns:
            The map dictionary of unique junction Id numbers, indexed by road graph keys.
        """
        junction_map = {}
        ctr = 0
        for i in range(len(path_segments)):
            seg = path_segments[i]
            key1 = seg[0]
            # The first node in a path segment is a junction node. Store it if we have not already found it.
            if key1 not in junction_map:
                junction_map[key1] = ctr
                ctr = ctr + 1
            key2 = seg[-1]
            # The last node in a path segment is also a junction node. Store it if we have not already found it.
            if key2 not in junction_map:
                junction_map[key2] = ctr
                ctr = ctr + 1
        return junction_map

    def _compute_tangents(self, seg):
        """
        Computes the tangent vector for every node in a given path segment.

        Args:
            seg: The given path segment.

        Returns:
            The tangents at every node in the given path segment.
        """
        tangents = []
        seg_length = len(seg)
        for j in range(seg_length):
            if j > 0 and j < seg_length - 1:
                p0 = self.coords[seg[max(j - 1, 0)]]
                p1 = self.coords[seg[j]]
                p2 = self.coords[seg[min(j + 1, seg_length - 1)]]
                d1 = max(math.sqrt(p0.distance(p1)), 1e-30)
                d2 = math.sqrt(p1.distance(p2))
                tangents.append((p1 - p0) * (d2 / d1) - (p2 - p0) * (d2 / (d1 + d2)) + (p2 - p1))
            elif j == 0:
                p1 = self.coords[seg[j]]
                p2 = self.coords[seg[min(j + 1, seg_length - 1)]]
                tangents.append(0.5 * (p2 - p1))
            else:
                p0 = self.coords[seg[max(j - 1, 0)]]
                p1 = self.coords[seg[j]]
                tangents.append(0.5 * (p1 - p0))
        return tangents

    def _fit_parametric_cubic(self, p1, p2, t_start, t_end):
        """
        Computes a parametric cubic which passes through the two points p1, p2, and matches tangents t_start, t_end.

        Args:
            p1: The start point.
            p2: The end point.
            t_start: The tangent at the start point, to which this cubic should fit itself.
            t_end: The tangent at the end point, to which this cubic should fit itself.

        Returns:
            The parametric cubic polynomial coefficients.
            Note: we ignore the two constant terms and the linear term of the 'v' equation.
        """
        # Compute the unit (s, t) coordinate system axes for this section.
        s = t_start.normalize()
        t = vec3(-s.y, s.x)

        # Compute points x1 and x2 in the (s, t) coordinate system reference space [0, 1]^2:
        p2norm = p2 - p1
        x2 = p2norm.dot(s)
        y2 = p2norm.dot(t)

        # Compute the end point tangent, in the (s, t) coordinate system.
        tan = vec3(t_end.dot(s), t_end.dot(t))

        # Compute the parametric cubic polynomial coefficients.
        Bu = t_start.length()
        Cu = (3.0 * x2) - tan.x - (2.0 * Bu)
        Du = (-2.0 * x2) + tan.x + Bu
        Cv = (3.0 * y2) - tan.y
        Dv = tan.y - (2.0 * y2)

        return parametric_cubic(Bu, Cu, Du, Cv, Dv)

    def compute_roads(self):
        """
        Computes a collection of individual roads, ready for export.
        """

        # Compute all the individual path segments from the loaded map.
        path_segments = self.compute_path_segments()

        # Compute a uniquely-identifiable list of roads (between two nodes).
        roads = []
        for i in range(len(path_segments)):
            seg = path_segments[i]

            # Compute the tangent vector at every node in this path segment.
            tangents = self._compute_tangents(seg)

            # Iterate over all sections of road in this path segment, in order.
            for j in range(len(seg) - 1):

                # Fetch the start and end points of this road section, and their two keys in the graph data structure.
                key1 = seg[j]
                key2 = seg[j + 1]
                p1 = self.coords[key1]
                p2 = self.coords[key2]

                cubic = self._fit_parametric_cubic(p1, p2, tangents[j], tangents[j + 1])

                # Create the road section.
                heading_angle = math.atan2(tangents[j].y, tangents[j].x)
                start_width = self.widths[key1]
                end_width = self.widths[key2]
                line_length = p1.distance(p2)
                line_length_inv = 1.0 / line_length
                linear_width = (end_width - start_width) * line_length_inv
                linear_elevation = (p2.z - p1.z) * line_length_inv
                roads.append(Road(key1, key2, p1, line_length, heading_angle,
                             self.coords[key1].z, linear_elevation, start_width, linear_width, cubic))

        # Create a map between junction key names and unique Id numbers.
        junction_map = self._graph_key_to_junction_map(path_segments)

        # Populate the remaining properties in the roads collection (successor road, predecessor road, junction, and contact point).
        for i in range(len(roads)):
            r = roads[i]
            predecessor = "none"
            successor = "none"
            junction = -1
            contact_point = "none"
            # Compute the successor and predecessor roads for this road, if they exist.
            for j in range(len(roads)):
                if i == j:
                    continue
                if r.start == roads[j].end:
                    predecessor = j
                if r.end == roads[j].start:
                    successor = j
            if r.start in junction_map:                 # Compute the junction and contact point data for this road, if they exist.
                junction = junction_map[r.start]
                contact_point = 'start'
            elif r.end in junction_map:
                junction = junction_map[r.end]
                contact_point = 'end'
            roads[i].update_connection_data(predecessor, successor, junction, contact_point)

        # Create a list of uniquely-identifiable junctions, containing all the connection road data relevant to them.
        junctions = []
        for k in junction_map.keys():
            connection_roads = []
            for i in range(len(roads)):
                if k == i:
                    continue
                r = roads[i]
                if k == r.start:
                    connection_roads.append(Connection('road', i, 'start'))
                elif k == r.end:
                    connection_roads.append(Connection('road', i, 'end'))
            is_end_point = len(connection_roads) == 0
            junctions.append(Junction(junction_map[k], connection_roads, is_end_point))

        return {'roads': roads, 'junctions': junctions}

    def export_xodr(self, name):
        """
        Exports the road network data to OpenDrive (.xodr) format.
        The export contains all road sections, some basic lane data, and some junction connectivity data.

        Args:
            name: The path/filename by which to save the .xodr file.
        """

        # Get the road data.
        road_data = self.compute_roads()
        roads = road_data['roads']
        junctions = road_data['junctions']

        # Write the road network data to .xodr format (xml).
        date_time = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        file_name = name + '.xodr'
        with open(file_name, 'w') as f:

            # .xodr file pre-amble.
            f.write('<?xml version="1.0" standalone="yes"?>\n')
            f.write('<OpenDRIVE>\n')
            f.write(
                '\t<header revMajor="1" revMinor="7" name="" version="1.00" date="' + date_time +
                '" north="0.0000000000000000e+00" south="0.0000000000000000e+00" east="0.0000000000000000e+00" west="0.0000000000000000e+00">\n')
            f.write('\t</header>\n')

            # Write the road data, in order.
            for i in range(len(roads)):
                r = roads[i]

                # Road header data.
                f.write('\t<road rule="RHT" length="' + str(r.length) + '" id="' +
                        str(i) + '" junction="' + str(r.junction) + '" >\n')

                # Road connectivity data.
                f.write('\t\t<link>\n')
                if r.predecessor != 'none':
                    f.write('\t\t\t<predecessor elementType="' + 'road' + '" elementId="' +
                            str(r.predecessor) + '" contactPoint="' + str(r.contact_point) + '" />\n')
                if r.successor != 'none':
                    f.write('\t\t\t<successor elementType="' + 'road' + '" elementId="' +
                            str(r.successor) + '" contactPoint="' + str(r.contact_point) + '" />\n')
                f.write('\t\t</link>\n')

                # Geometry data.
                f.write('\t\t<type s="0.0000000000000000e+00" type="town" country="DE"/>\n')
                f.write('\t\t<planView>\n')
                f.write('\t\t\t<geometry s="0.0000000000000000e+00" x="' + str(r.p1.x) + '" y="' +
                        str(r.p1.y) + '" hdg="' + str(r.hdg) + '" length="' + str(r.length) + '">\n')
                f.write('\t\t\t\t<paramPoly3 aU="0.0000000000000000e+00" bU="' + str(r.Bu) + '" cU="' + str(r.Cu) +
                        '" dU="' + str(r.Du) + '" aV="0.0000000000000000e+00" bV="0.0000000000000000e+00" cV="' +
                        str(r.Cv) + '" dV="' + str(r.Dv) + '"/>\n')
                f.write('\t\t\t</geometry>\n')
                f.write('\t\t</planView>\n')

                # Elevation data.
                f.write('\t\t<elevationProfile>\n')
                f.write('\t\t\t<elevation s="0.0000000000000000e+00" a="' + str(r.start_elevation) + '" b="' +
                        str(r.linear_elevation) + '" c="0.0000000000000000e+00" d="0.0000000000000000e+00"/>\n')
                f.write('\t\t</elevationProfile>\n')
                f.write('\t\t<lateralProfile>\n')
                f.write('\t\t</lateralProfile>\n')

                # Road lane data.
                f.write('\t\t<lanes>\n')
                f.write('\t\t\t<laneSection s="0.0000000000000000e+00">\n')
                f.write('\t\t\t\t<left>\n')
                f.write('\t\t\t\t\t<lane id="1" type="driving" level="false">\n')
                f.write('\t\t\t\t\t\t<link>\n')
                f.write('\t\t\t\t\t\t</link>\n')
                f.write('\t\t\t\t\t\t<width sOffset="0.0000000000000000e+00" a="' + str(r.start_width) +
                        '" b="' + str(r.linear_width) + '" c="0.0000000000000000e+00" d="0.0000000000000000e+00"/>\n')
                f.write('\t\t\t\t\t</lane>\n')
                f.write('\t\t\t\t</left>\n')
                f.write('\t\t\t\t<center>\n')
                f.write('\t\t\t\t\t<lane id="0" type="driving" level="false">\n')
                f.write('\t\t\t\t\t\t<link>\n')
                f.write('\t\t\t\t\t\t</link>\n')
                f.write('\t\t\t\t\t\t<roadMark sOffset="2.0000000000000000e+00" type="broken" weight="standard" color="standard" width="1.2000000000000000e-01" laneChange="both" height="1.9999999552965164e-02">\n')
                f.write('\t\t\t\t\t\t\t<type name="broken" width="1.2000000000000000e-01">\n')
                f.write('\t\t\t\t\t\t\t\t<line length="3.0000000000000000e+00" space="6.0000000000000000e+00" tOffset="0.0000000000000000e+00" sOffset="0.0000000000000000e+00" rule="caution" width="1.2000000000000000e-01"/>\n')
                f.write('\t\t\t\t\t\t\t</type>\n')
                f.write('\t\t\t\t\t\t</roadMark>\n')
                f.write('\t\t\t\t\t</lane>\n')
                f.write('\t\t\t\t</center>\n')
                f.write('\t\t\t\t<right>\n')
                f.write('\t\t\t\t\t<lane id="-1" type="driving" level="false">\n')
                f.write('\t\t\t\t\t\t<link>\n')
                f.write('\t\t\t\t\t\t</link>\n')
                f.write('\t\t\t\t\t\t<width sOffset="0.0000000000000000e+00" a="' + str(r.start_width) +
                        '" b="' + str(r.linear_width) + '" c="0.0000000000000000e+00" d="0.0000000000000000e+00"/>\n')
                f.write('\t\t\t\t\t</lane>\n')
                f.write('\t\t\t\t</right>\n')
                f.write('\t\t\t</laneSection>\n')
                f.write('\t\t</lanes>\n')

                # TODO : WE DO NOT CURRENTLY MAKE USE OF THESE SECTIONS.
                f.write('\t\t<objects>\n')
                f.write('\t\t</objects>\n')
                f.write('\t\t<signals>\n')
                f.write('\t\t</signals>\n')
                f.write('\t\t<surface>\n')
                f.write('\t\t</surface>\n')

                f.write('\t</road>\n')

            # Write the junction data, in order.
            for i in range(len(junctions)):
                jct = junctions[i]
                f.write('\t<junction name="" id="' + str(jct.id) + '" type="default">\n')
                ctr = 0
                for j in range(len(jct.connection_roads)):
                    a = jct.connection_roads[j]
                    for k in range(len(jct.connection_roads)):
                        b = jct.connection_roads[k]
                        f.write('\t\t<connection id="' + str(ctr) + '" incomingRoad="' + str(a.id) +
                                '" connectingRoad="' + str(b.id) + '" contactPoint="' + str(a.contact_point) + '">\n')
                        if a.contact_point == 'start':
                            f.write('\t\t\t<laneLink from="-1" to="1"/>\n')
                        else:
                            f.write('\t\t\t<laneLink from="1" to="-1"/>\n')
                        f.write('\t\t</connection>\n')
                        ctr = ctr + 1
                f.write('\t</junction>\n')

            f.write('</OpenDRIVE>\n')

    def export_osm(self, name):
        """
        TODO: THIS EXPORTER IS A WORK IN PROGRESS AND MAY EXHIBIT SOME INCORRECT BEHAVIOUR.
        Exports the road network data to OpenStreetMap (.osm) format.
        The export contains all road sections, some basic lane data, and some junction connectivity data.

        Args:
            name: The path/filename by which to save the .osm file.
        """

        # Compute all the individual path segments from the loaded map.
        path_segments = self.compute_path_segments()

        # Create the nodes data: A unique list of nodes, a map from graph keys to unique node id, and bounds info.
        scale_factor = 1.0 / 1e7  # to convert metres into reasonable lattitude/longitude values.
        nodes = []
        keys_to_node_map = {}
        minlat = 1e99
        minlon = 1e99
        maxlat = -1e99
        maxlon = -1e99
        ctr = 0
        for k, v in self.coords.items():
            keys_to_node_map[k] = ctr
            coord = vec3(v.x * scale_factor + 45.0, v.y * scale_factor + 45.0, v.z)
            nodes.append(coord)
            minlat = min(minlat, coord.x)
            minlon = min(minlon, coord.y)
            maxlat = max(maxlat, coord.x)
            maxlon = max(maxlon, coord.y)
            ctr = ctr + 1

        # Create the unique list of OpenStreetMap 'ways'.
        ways = []
        for _, seg in path_segments.items():
            n = []
            for i in range(len(seg)):
                n.append(keys_to_node_map[seg[i]])
            ways.append(n)

        # Write the road network data to .osm format (xml).
        file_name = name + '.osm'
        date_time = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        with open(file_name, 'w') as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            f.write('<osm version="0.6" generator="BeamNGPy">\n')
            f.write('\t<bounds minlat="' + str(minlat) + '" minlon="' + str(minlon) +
                    '" maxlat="' + str(maxlat) + '" maxlon="' + str(maxlon) + '"/>\n')
            for i in range(len(nodes)):
                nodeId = i + 1
                lat = str(nodes[i][0])
                lon = str(nodes[i][1])
                ele = str(nodes[i][2])
                f.write('\t<node id="' + str(nodeId) + '" lat="' + lat + '" lon="' + lon + '" ele="' + ele +
                        '" user="BeamNG" uid="1" visible="true" version="1" changeset="1" timestamp="' + date_time + '"/>\n')
            for i in range(len(ways)):
                wayId = i + 1  # the OpenStreetMap Id numbers start at 1 not 0.
                f.write('\t<way id="' + str(wayId) + '" user="BeamNG" uid="1" visible="true" version="1" changeset="1">\n')
                seg = ways[i]
                for j in range(len(seg)):
                    nodeId = seg[j] + 1
                    f.write('\t\t<nd ref="' + str(nodeId) + '"/>\n')
                f.write('\t</way>\n')
            f.write('</osm>\n')

    def __init__(self, bng: BeamNGpy):
        """
        Creates an instance of the BeamNGpy OpenDrive exporter.

        Args:
            bng: The BeamNG instance.
        """
        self.logger = getLogger(f'{LOGGER_ID}.Road_Graph')
        self.logger.setLevel(DEBUG)
        self.bng = bng

        # Get the road graph data for the current map.
        raw_data = self._send_sensor_request('GetRoadGraph', ack='CompletedGetRoadGraph')['data']
        self.graph = raw_data['graph']
        self.coords = self._to_vec3(raw_data['coords'])
        self.widths = raw_data['widths']
        self.normals = raw_data['normals']
        self.logger.debug('Road_Graph - data retrieved.')

    def _send_sensor_request(self, type: str, ack: str | None = None, **kwargs: Any) -> StrDict:
        if not self.bng.connection:
            raise BNGError('The simulator is not connected!')
        return send_sensor_request(self.bng.connection, type, ack, **kwargs)

    def _set_sensor(self, type: str, **kwargs: Any) -> None:
        if not self.bng.connection:
            raise BNGError('The simulator is not connected!')
        set_sensor(self.bng.connection, type, **kwargs)

    def plot_path_segments(self, path_segments):
        """
        Displays the individual path segments.
        Each junction node is highlighted in red.  Each intermediate path node is highlighted in blue.

        Args:
            path_segments: The collection of individual 'path segments' to plot.
        """
        sns.set()

        fig, ax = plt.subplots(figsize=(15, 15))
        px = []
        py = []
        lines = []
        line_colors = []
        node_colors = []
        for i in range(len(path_segments)):
            seg = path_segments[i]
            px.append(self.coords[seg[0]][0])
            py.append(self.coords[seg[0]][1])
            node_colors.append((1.0, 0.0, 0.0, 1.0))
            for j in range(1, len(seg)):
                px.append(self.coords[seg[j]][0])
                py.append(self.coords[seg[j]][1])
                node_colors.append((.0, 0.0, 1.0, 1.0))
                p1 = self.coords[seg[j - 1]]
                p2 = self.coords[seg[j]]
                lines.append([(p1[0], p1[1]), (p2[0], p2[1])])
                line_colors.append((0.3, 0.3, 0.3, 0.5))
            node_colors[-1] = (1.0, 0.0, 0.0, 1.0)

        ax.add_collection(mc.LineCollection(lines, colors=line_colors, linewidths=0.5))
        ax.scatter(px, py, s=3.0, c=node_colors, cmap=matplotlib.cm.viridis)
        plt.show()
