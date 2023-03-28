from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any

from beamngpy.logging import LOGGER_ID, BNGError
from beamngpy.types import StrDict

from beamngpy.sensors.communication_utils import send_sensor_request, set_sensor

import math

from datetime import datetime

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import collections  as mc

import seaborn as sns
sns.set()

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy

__all__ = ['Opendrive_Exporter']

class Connection:
    def __init__(self, connection_type, id, contact_point):
        self.connection_type = connection_type
        self.id = id
        self.contact_point = contact_point

class Road:
    def __init__(self, start, end, start_x, start_y, length, hdg, start_elevation, linear_elevation, start_width, linear_width, Cu, Du, Cv, Dv, Bu,
        predecessor = None, successor = None, junction = None, contact_point = None):
        self.start = start
        self.end = end
        self.start_x = start_x
        self.start_y = start_y
        self.hdg = hdg
        self.start_elevation = start_elevation
        self.linear_elevation = linear_elevation
        self.start_width = start_width
        self.linear_width = linear_width
        self.length = length
        self.Cu = Cu
        self.Du = Du
        self.Cv = Cv
        self.Dv = Dv
        self.Bu = Bu
        self.predecessor = predecessor
        self.successor = successor
        self.junction = junction
        self.contact_point = contact_point

class Junction:
    def __init__(self, id, connection_roads, is_end_point):
        self.id = id
        self.connection_roads = connection_roads
        self.is_end_point = is_end_point

class Opendrive_Exporter:
    """
    A class for retrieving the road graph data from the simulator.
    Args:
        bng: The BeamNGpy instance, with which to communicate to the simulation.
    """

    def _normalize(self, v) -> list[float]:
        mag_inv = 1.0 / math.sqrt(v[0] * v[0] + v[1] * v[1])
        return [v[0] * mag_inv, v[1] * mag_inv]

    def _dot(self, v1, v2) -> float:
        return v1[0] * v2[0] + v1[1] * v2[1]

    def _L2(self, a, b) -> float:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        return math.sqrt(dx * dx + dy * dy)

    def _collection_does_not_contain_segment(self, collection, segment) -> bool:
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
        collection = {}
        ctr = 0
        for head_key in self.graph.keys():
            children = self.graph[head_key].keys()
            if len(children) != 2:
                for child_key in children:
                    current_path = []
                    current_path.append(head_key)
                    next_key = child_key
                    while(True):
                        current_path.append(next_key)
                        next_children = self.graph[next_key].keys()
                        if len(next_children) != 2:
                            if self._collection_does_not_contain_segment(collection, current_path):
                                collection[ctr] = current_path
                                ctr = ctr + 1
                            break
                        did_find = False
                        for grandchild_key in next_children:
                            if grandchild_key not in current_path:
                                next_key = grandchild_key
                                did_find = True
                                break
                        if did_find == False:
                            if self._collection_does_not_contain_segment(collection, current_path):
                                collection[ctr] = current_path
                                ctr = ctr + 1
                            break
        return collection

    def export_xodr(self, name):

        # Compute all the individual path segments from the loaded map.
        path_segments = self.compute_path_segments()

        # Compute a uniquely-identifiable list of roads (between two nodes).
        roads = []
        for i in range(len(path_segments)):
            seg = path_segments[i]
            seg_length = len(seg)

            # Compute the tangent vector at every node in this path segment.
            tangents = []
            for j in range(seg_length):
                if j == 0:
                    pk1 = [self.coords[seg[j]][0], self.coords[seg[j]][1]]
                    pk2 = [self.coords[seg[j + 1]][0], self.coords[seg[j + 1]][1]]
                    numx = 0.5 * (pk2[0] - pk1[0])
                    numy = 0.5 * (pk2[1] - pk1[1])
                    inv_denom = 1.0 / (math.sqrt(self._L2(pk2, pk1)))
                    tan = [numx * inv_denom, numy * inv_denom]
                elif j == seg_length - 1:
                    pk0 = [self.coords[seg[j - 1]][0], self.coords[seg[j - 1]][1]]
                    pk1 = [self.coords[seg[j]][0], self.coords[seg[j]][1]]
                    numx = 0.5 * (pk1[0] - pk0[0])
                    numy = 0.5 * (pk1[1] - pk0[1])
                    inv_denom = 1.0 / math.sqrt(self._L2(pk1, pk0))
                    tan = [numx * inv_denom, numy * inv_denom]
                else:
                    pk0 = [self.coords[seg[j - 1]][0], self.coords[seg[j - 1]][1]]
                    pk1 = [self.coords[seg[j]][0], self.coords[seg[j]][1]]
                    pk2 = [self.coords[seg[j + 1]][0], self.coords[seg[j + 1]][1]]
                    numx = 0.5 * (pk2[0] - pk0[0])
                    numy = 0.5 * (pk2[1] - pk0[1])
                    inv_denom = 1.0 / (math.sqrt(self._L2(pk2, pk1)) + math.sqrt(self._L2(pk1, pk0)))
                    tan = [numx * inv_denom, numy * inv_denom]
                tangents.append(tan)

            # Iterate over all sections of road in this path segment, in order.
            for j in range(seg_length - 1):

                # Compute the unit (s, t) coordinate system axes for this section.
                s = self._normalize(tangents[j])
                t = [-s[1], s[0]]

                # Compute points x1 and x2 in the (s, t) coordinate system reference space [0, 1]^2:
                k = j + 1
                x1_world = self.coords[seg[j]][0]
                y1_world = self.coords[seg[j]][1]
                x2_world = self.coords[seg[k]][0]
                y2_world = self.coords[seg[k]][1]
                p1_to_p2 = [x2_world - x1_world, y2_world - y1_world]
                x2 = self._dot(p1_to_p2, s)
                y2 = self._dot(p1_to_p2, t)

                # Compute the end point tangents tan2, in the (s, t) coordinate system.
                tan2x = self._dot(tangents[k], s)
                tan2y = self._dot(tangents[k], t)
                tan1mag = math.sqrt(tangents[j][0] * tangents[j][0] + tangents[j][1] * tangents[j][1])

                # Compute the parametric cubic polynomial curve coefficients.
                Cu = 3.0 * x2 - tan2x - 2.0 * tan1mag
                Du = -2.0 * x2 + tan2x + tan1mag
                Cv = 3.0 * y2 - tan2y
                Dv = tan2y - 2.0 * y2

                # Create the road section.
                hdg = math.atan2(tangents[j][1], tangents[j][0])
                a = seg[j]
                b = seg[k]
                start_width = self.widths[a]
                end_width = self.widths[b]
                dx = x2_world - x1_world
                dy = y2_world - y1_world
                line_length = math.sqrt(dx * dx + dy * dy)
                linear_width = (end_width - start_width) / line_length
                linear_elevation = (self.coords[b][2] - self.coords[a][2]) / line_length
                roads.append(Road(a, b, x1_world, y1_world, line_length, hdg, self.coords[a][2], linear_elevation, start_width, linear_width, Cu, Du, Cv, Dv, tan1mag))

        # Create a map between junction key names and unique Id numbers.
        junction_map = {}
        ctr = 0
        for i in range(len(path_segments)):
            seg = path_segments[i]
            a = seg[0]
            if a not in junction_map:   # The first node in a path segment is a junction node. Store it if we have not already found it.
                junction_map[a] = ctr
                ctr = ctr + 1
            b = seg[-1]
            if b not in junction_map:   # The last node in a path segment is also a junction node. Store it if we have not already found it.
                junction_map[b] = ctr
                ctr = ctr + 1

        # Populate the remaining properties in the roads collection (successor road, predecessor road, junction, and contact point).
        for i in range(len(roads)):
            r = roads[i]
            predecessor = "none"
            successor = "none"
            junction = -1
            contact_point = "none"
            for j in range(len(roads)):                 # Compute the successor and predecessor roads (for this road), if they exist.
                if i == j:
                    continue
                tr = roads[j]
                if r.start == tr.end:
                    predecessor = j
                if r.end == tr.start:
                    successor = j
            if r.start in junction_map:                 # Compute the junction and contact point data for this road, if they exist.
                junction = junction_map[r.start]
                contact_point = 'start'
            elif r.end in junction_map:
                junction = junction_map[r.end]
                contact_point = 'end'
            roads[i].predecessor = predecessor
            roads[i].successor = successor
            roads[i].junction = junction
            roads[i].contact_point = contact_point

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

        # Write to .xodr format (xml).
        date_time = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        file_name = name + '.xodr'
        with open(file_name, 'w') as f:
            f.write('<?xml version="1.0" standalone="yes"?>\n')
            f.write('<OpenDRIVE>\n')
            f.write('\t<header revMajor="1" revMinor="7" name="" version="1.00" date="' + date_time + '" north="0.0000000000000000e+00" south="0.0000000000000000e+00" east="0.0000000000000000e+00" west="0.0000000000000000e+00">\n')
            f.write('\t</header>\n')

            for i in range(len(roads)):
                r = roads[i]
                f.write('\t<road rule="RHT" length="' + str(r.length) + '" id="' + str(i) + '" junction="' + str(r.junction) + '" >\n')

                f.write('\t\t<link>\n')
                if r.predecessor != 'none':
                    f.write('\t\t\t<predecessor elementType="' + 'road' + '" elementId="' + str(r.predecessor) + '" contactPoint="' + str(r.contact_point) + '" />\n')
                if r.successor != 'none':
                    f.write('\t\t\t<successor elementType="' + 'road' + '" elementId="' + str(r.successor) + '" contactPoint="' + str(r.contact_point) + '" />\n')
                f.write('\t\t</link>\n')

                f.write('\t\t<type s="0.0000000000000000e+00" type="town" country="DE"/>\n')
                f.write('\t\t<planView>\n')
                f.write('\t\t\t<geometry s="0.0000000000000000e+00" x="' + str(r.start_x) + '" y="' + str(r.start_y) + '" hdg="' + str(r.hdg) + '" length="' + str(r.length) + '">\n')
                aU = str(0.0)
                bU = str(r.Bu)
                cU = str(r.Cu)
                dU = str(r.Du)
                aV = str(0.0)
                bV = str(0.0)
                cV = str(r.Cv)
                dV = str(r.Dv)
                f.write('\t\t\t\t<paramPoly3 aU="' + aU + '" bU="' + bU + '" cU="' + cU + '" dU="' + dU + '" aV="' + aV + '" bV="' + bV + '" cV="' + cV + '" dV="' + dV +'"/>\n')
                f.write('\t\t\t</geometry>\n')
                f.write('\t\t</planView>\n')

                f.write('\t\t<elevationProfile>\n')
                f.write('\t\t\t<elevation s="0.0000000000000000e+00" a="' + str(r.start_elevation) + '" b="' + str(r.linear_elevation) + '" c="0.0000000000000000e+00" d="0.0000000000000000e+00"/>\n')
                f.write('\t\t</elevationProfile>\n')
                f.write('\t\t<lateralProfile>\n')
                f.write('\t\t</lateralProfile>\n')

                f.write('\t\t<lanes>\n')
                f.write('\t\t\t<laneSection s="0.0000000000000000e+00">\n')
                f.write('\t\t\t\t<left>\n')
                f.write('\t\t\t\t\t<lane id="1" type="driving" level="false">\n')
                f.write('\t\t\t\t\t\t<link>\n')
                f.write('\t\t\t\t\t\t</link>\n')
                f.write('\t\t\t\t\t\t<width sOffset="0.0000000000000000e+00" a="' + str(r.start_width) + '" b="' + str(r.linear_width) + '" c="0.0000000000000000e+00" d="0.0000000000000000e+00"/>\n')
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
                f.write('\t\t\t\t\t\t<width sOffset="0.0000000000000000e+00" a="' + str(r.start_width) + '" b="' + str(r.linear_width) + '" c="0.0000000000000000e+00" d="0.0000000000000000e+00"/>\n')
                f.write('\t\t\t\t\t</lane>\n')
                f.write('\t\t\t\t</right>\n')
                f.write('\t\t\t</laneSection>\n')
                f.write('\t\t</lanes>\n')

                f.write('\t\t<objects>\n')
                f.write('\t\t</objects>\n')
                f.write('\t\t<signals>\n')
                f.write('\t\t</signals>\n')
                f.write('\t\t<surface>\n')
                f.write('\t\t</surface>\n')

                f.write('\t</road>\n')

            for i in range(len(junctions)):
                jct = junctions[i]
                f.write('\t<junction name="" id="' + str(jct.id) + '" type="default">\n')
                ctr = 0
                for j in range(len(jct.connection_roads)):
                    a = jct.connection_roads[j]
                    for k in range(len(jct.connection_roads)):
                        b = jct.connection_roads[k]
                        f.write('\t\t<connection id="' + str(ctr) + '" incomingRoad="' + str(a.id) + '" connectingRoad="' + str(b.id) + '" contactPoint="' + str(a.contact_point) + '">\n')
                        if a.contact_point == 'start':
                            f.write('\t\t\t<laneLink from="-1" to="1"/>\n')
                        else:
                            f.write('\t\t\t<laneLink from="1" to="-1"/>\n')
                        f.write('\t\t</connection>\n')
                        ctr = ctr + 1
                f.write('\t</junction>\n')

            f.write('</OpenDRIVE>\n')

    def __init__(self, bng: BeamNGpy):
        self.logger = getLogger(f'{LOGGER_ID}.Road_Graph')
        self.logger.setLevel(DEBUG)
        self.bng = bng

        # Get the road graph data for the current map.
        raw_data = self._send_sensor_request('GetRoadGraph', ack='CompletedGetRoadGraph')['data']
        self.graph = raw_data['graph']
        self.coords = raw_data['coords']
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
