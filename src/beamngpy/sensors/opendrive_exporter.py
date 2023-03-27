from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any

from beamngpy.logging import LOGGER_ID, BNGError
from beamngpy.types import StrDict

from .communication_utils import send_sensor_request, set_sensor

import beamngpy.circle as crc
import beamngpy.cubic as cub

import math
import numpy as np

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
    def __init__(self, type, start, end, predecessor, successor, start_x, start_y, length, start_k, end_k, start_elevation, linear_elevation, start_width, linear_width,
        hdg, junction, contact_point, cubic):
        self.type = type
        self.start = start
        self.end = end
        self.predecessor = predecessor
        self.successor = successor
        self.start_x = start_x
        self.start_y = start_y
        self.length = length
        self.start_k = start_k
        self.end_k = end_k
        self.start_elevation = start_elevation
        self.linear_elevation = linear_elevation
        self.start_width = start_width
        self.linear_width = linear_width
        self.hdg = hdg
        self.junction = junction
        self.contact_point = contact_point
        if cubic is not None:
            self.aU = cubic.a
            self.bU = cubic.b
            self.cU = cubic.c
            self.dU = cubic.d

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

    def _get_plan(self, i) -> list[str]:

        # Build a path using repeating blocks, then add a final remainder section.
        common = ['c0', 'c1', 's']
        remainder = {}
        remainder[2] = ['c0', 'c1']
        remainder[1] = ['c1']
        f = math.floor(i / 3)
        r = i % 3
        build = []
        for i in range(f):
            build += common

        if r == 0:
            return build
        return build + remainder[r]

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

    def compute_path_segments(self):
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

        # We want to ensure the minimum length of a road segment == 3 (instead of 2, which it currently is) so we split segments of size two in half.
        ctr = 0
        processed_collection = {}
        for k, seg in collection.items():
            if len(seg)  < 2:
                print("WARNING!!  ROAD SEGMENT WITH LESS THAN 2 NODES FOUND.  REMOVED", k, seg)
            elif len(seg) == 2:
                name = 'added_node_' + str(ctr)
                self.graph[name] = {}                                                       # Create a new node in the graph.
                self.graph[name][seg[0]] = self.graph[seg[0]]
                self.graph[name][seg[1]] = self.graph[seg[1]]
                mid_x = (self.coords[seg[0]][0] + self.coords[seg[1]][0]) * 0.5
                mid_y = (self.coords[seg[0]][1] + self.coords[seg[1]][1]) * 0.5
                mid_z = (self.coords[seg[0]][2] + self.coords[seg[1]][2]) * 0.5
                self.coords[name] = [mid_x, mid_y, mid_z]                                   # Store the new coordinates of the node (use the midpoint).
                self.widths[name] = (self.widths[seg[0]] + self.widths[seg[1]]) * 0.5       # Store the new width (use width average).
                n_x = (self.normals[seg[0]][0] + self.normals[seg[1]][0]) * 0.5
                n_y = (self.normals[seg[0]][1] + self.normals[seg[1]][1]) * 0.5
                n_z = (self.normals[seg[0]][2] + self.normals[seg[1]][2]) * 0.5
                self.normals[name] = [n_x, n_y, n_z]                                        # Store the new surface normal (use the average).
                new_road_segment = [seg[0], name, seg[1]]                                   # Create the new road segment.
                processed_collection[ctr] = new_road_segment
                ctr = ctr + 1
            else:
                processed_collection[ctr] = seg                                             # Nothing changes with any road segments with more than 2 nodes.
                ctr = ctr + 1
        return processed_collection

    def fit_line(self, j, k, seg, x_start, y_start, x_end, y_end) -> Road:
        a = seg[j]
        b = seg[k]
        dx = x_end - x_start
        dy = y_end - y_start
        length = math.sqrt(dx * dx + dy * dy)                                   # road length (only use x and y - elevation is separate).
        hdg = math.atan2(dy, dx)                                                # heading (rad).
        start_width = self.widths[a]
        end_width = self.widths[b]
        linear_width = (end_width - start_width) / length                       # linear width rate across road.
        linear_elevation = (self.coords[b][2] - self.coords[a][2]) / length     # linear elevation rate across road.
        return Road('line', a, b, None, None, x_start, y_start, length, 0.0, 0.0, self.coords[a][2], linear_elevation, start_width, linear_width, hdg, None, None, None)

    def fit_arc(self, j, k, seg, circle, x_start, y_start, x_end, y_end) -> Road:
        a = seg[j]
        b = seg[k]
        length = circle.arc_length(x_start, y_start, x_end, y_end)
        hdg = circle.hdg(x_start, y_start, x_end, y_end)
        start_width = self.widths[a]
        end_width = self.widths[b]
        linear_width = (end_width - start_width) / length                       # linear width rate across road.
        linear_elevation = (self.coords[b][2] - self.coords[a][2]) / length     # linear elevation rate across road.
        return Road('arc', a, b, None, None, x_start, y_start, length, circle.k, 0.0, self.coords[a][2], linear_elevation, start_width, linear_width, hdg, None, None, None)

    def fit_spiral(self, j, k, seg, hdg, k_start, k_end, length, x_start, y_start) -> Road:
        a = seg[j]
        b = seg[k]
        start_width = self.widths[a]
        end_width = self.widths[b]
        linear_width = (end_width - start_width) / length                       # linear width rate across road.
        linear_elevation = (self.coords[b][2] - self.coords[a][2]) / length     # linear elevation rate across road.
        return Road('spiral', a, b, None, None, x_start, y_start, length, k_start, k_end, self.coords[a][2], linear_elevation, start_width, linear_width, hdg, None, None, None)

    def fit_cubic(self, j, k, seg, hdg, x_start, y_start, length, cubic) -> Road:
        a = seg[j]
        b = seg[k]
        start_width = self.widths[a]
        end_width = self.widths[b]
        linear_width = (end_width - start_width) / length                       # linear width rate across road.
        linear_elevation = (self.coords[b][2] - self.coords[a][2]) / length     # linear elevation rate across road.
        return Road('cubic', a, b, None, None, x_start, y_start, length, None, None, self.coords[a][2], linear_elevation, start_width, linear_width, hdg, None, None, cubic)

    def export_xodr_curved(self, name):

        # Compute all the individual path segments from the loaded map.
        path_segments = self.compute_path_segments()

        # Compute a uniquely-identifiable list of roads (between two nodes).
        roads = []
        for i in range(len(path_segments)):
            seg = path_segments[i]
            seg_length = len(seg)
            plan = self._get_plan(seg_length - 1)   # Get a template curve-fitting plan for this path segment.

            # Iterate over all sections of road in this path segment, in order.
            for j in range(seg_length - 1):
                x_start = self.coords[seg[j]][0]
                y_start = self.coords[seg[j]][1]
                x_end = self.coords[seg[j + 1]][0]
                y_end = self.coords[seg[j + 1]][1]
                if plan[j] == 'c0':                 # into this section [j, j+1], attempt to fit the first half of a circle arc.
                    n1 = seg[j]
                    n2 = seg[j + 1]
                    n3 = seg[j + 2]
                    x1 = self.coords[n1][0]
                    y1 = self.coords[n1][1]
                    x2 = self.coords[n2][0]
                    y2 = self.coords[n2][1]
                    x3 = self.coords[n3][0]
                    y3 = self.coords[n3][1]
                    (circle, is_collinear) = crc.circle.circle_from_3_points(x1, y1, x2, y2, x3, y3)
                    if is_collinear == True:
                        roads.append(self.fit_line(j, j + 1, seg, x_start, y_start, x_end, y_end))  # if the circle points are collinear, fit a line instead.
                        continue
                    roads.append(self.fit_arc(j, j + 1, seg, circle, x_start, y_start, x_end, y_end))
                elif plan[j] == 'c1':               # into this section [j, j+1], attempt to fit the second half of a circle arc.
                    n1 = seg[j - 1]
                    n2 = seg[j]
                    n3 = seg[j + 1]
                    x1 = self.coords[n1][0]
                    y1 = self.coords[n1][1]
                    x2 = self.coords[n2][0]
                    y2 = self.coords[n2][1]
                    x3 = self.coords[n3][0]
                    y3 = self.coords[n3][1]
                    (circle, is_collinear) = crc.circle.circle_from_3_points(x1, y1, x2, y2, x3, y3)
                    if is_collinear == True:
                        roads.append(self.fit_line(j, j + 1, seg, x_start, y_start, x_end, y_end))  # if the circle points are collinear, fit a line instead.
                        continue
                    roads.append(self.fit_arc(j, j + 1, seg, circle, x_start, y_start, x_end, y_end))
                elif plan[j] == 's':
                    # Get predecessor circle curvature (previous plan value must be 'c1'). A predecessor value must exist, since plans cannot start on 's'.
                    n1 = seg[j - 2]
                    n2 = seg[j - 1]
                    n3 = seg[j]
                    x1 = self.coords[n1][0]
                    y1 = self.coords[n1][1]
                    x2 = self.coords[n2][0]
                    y2 = self.coords[n2][1]
                    x3 = self.coords[n3][0]
                    y3 = self.coords[n3][1]
                    (pre_circle, is_pre_collinear) = crc.circle.circle_from_3_points(x1, y1, x2, y2, x3, y3)

                    # Get successor circle curvature (next plan value can be either be 'c0' or 'c1'). A successor value must exist, since plans cannot end on 's'.
                    post_circle = None
                    is_post_collinear = True
                    if j + 1 < len(plan):
                        if plan[j + 1] == 'c0':
                            n1 = seg[j + 1]
                            n2 = seg[j + 2]
                            n3 = seg[j + 3]
                            x1 = self.coords[n1][0]
                            y1 = self.coords[n1][1]
                            x2 = self.coords[n2][0]
                            y2 = self.coords[n2][1]
                            x3 = self.coords[n3][0]
                            y3 = self.coords[n3][1]
                            (post_circle, is_post_collinear) = crc.circle.circle_from_3_points(x1, y1, x2, y2, x3, y3)
                        elif plan[j + 1] == 'c1':
                            n1 = seg[j]
                            n2 = seg[j + 1]
                            n3 = seg[j + 2]
                            x1 = self.coords[n1][0]
                            y1 = self.coords[n1][1]
                            x2 = self.coords[n2][0]
                            y2 = self.coords[n2][1]
                            x3 = self.coords[n3][0]
                            y3 = self.coords[n3][1]
                            (post_circle, is_post_collinear) = crc.circle.circle_from_3_points(x1, y1, x2, y2, x3, y3)
                        else:
                            print("*** ERROR ***  - successor circle not found in plan")
                    else:
                        # In this case, we are at the end of the plan, and require a spiral to go from a circle to zero curvature. So we use a dummy circle with k=0.
                        #(post_circle, is_post_collinear) = Circle.circle_from_3_points(0, 1, 4, 9, 2, 6)
                        #post_circle.k = 0.0
                        roads.append(self.fit_line(j, j + 1, seg, x_start, y_start, x_end, y_end))
                        continue

                    if (is_pre_collinear == True or abs(pre_circle.k) < 1e-7) and (is_post_collinear == True or abs(post_circle.k) < 1e-7):
                        roads.append(self.fit_line(j, j + 1, seg, x_start, y_start, x_end, y_end))  # if both circle curvatures are ~ 0 then fit line instead of spiral.
                        continue

                    x_next = x_end
                    y_next = y_end
                    if j + 2 < seg_length:
                        x_next = self.coords[seg[j + 2]][0]
                        y_next = self.coords[seg[j + 2]][1]

                    xar = x_end - x_start
                    yar = y_end - y_start
                    mag = math.sqrt((x_end-x_start)*(x_end-x_start) + (y_end-y_start)*(y_end-y_start))
                    xar /= mag
                    yar /= mag
                    mat = np.array([[xar, yar], [-yar, xar]])                                           # rot matrix to rotate a line segment to the positive x-axis.

                    xaxis = np.array([1.0, 0.0])
                    rotxaxis = np.matmul(mat, xaxis)
                    ang = math.atan2(rotxaxis[1], rotxaxis[0])                                          # angle of rotation in radians.

                    slope0 = math.tan(pre_circle.hdg(x_start, y_start, x_end, y_end) + ang)             # add angle of rotation to the slopes.
                    slope1 = math.tan(post_circle.hdg(x_end, y_end, x_next, y_next) + ang)

                    v = np.array([x_end - x_start, y_end - y_start])
                    tres = np.matmul(mat, v)                                                            # Rotate the vector.
                    x2 = tres[0]
                    y2 = tres[1]

                    a = (slope0 + slope1 - 2.0 * (y2 / x2)) / (x2 * x2)
                    b = ((slope1 - slope0) / (2.0 * x2)) - (1.5 * a * x2)
                    cubic = cub.cubic(0.0, slope0, b, a)

                    is_end_fitted = abs(cubic.eval_upper_only(x2) - y2) < 8.0
                    hdg = pre_circle.hdg(x_start, y_start, x_end, y_end)
                    length = cubic.approx_length(0.0, x2)
                    dx = x_end - x_start
                    dy = y_end - y_start
                    straight_line_length = math.sqrt(dx * dx + dy * dy)
                    if is_end_fitted and length < (1.4 * straight_line_length):
                        roads.append(self.fit_cubic(j, j + 1, seg, hdg, x_start, y_start, length, cubic))
                        continue
                    roads.append(self.fit_line(j, j + 1, seg, x_start, y_start, x_end, y_end))

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
                if r.type == 'line':
                    f.write('\t\t\t\t<line/>\n')
                elif r.type == 'arc':
                    f.write('\t\t\t\t<arc curvature="' + str(r.start_k) + '"/>\n')
                elif r.type == 'spiral':
                    f.write('\t\t\t\t<spiral curvStart="' + str(r.start_k) + '" curvEnd="' + str(r.end_k) + '"/>\n')
                elif r.type == 'cubic':
                    aV = str(0.0)
                    bV = str(0.0)
                    cV = str(r.cU)
                    dV = str(r.dU)
                    aU = str(0.0)
                    bU = str(1.0)
                    cU = str(0.0)
                    dU = str(0.0)
                    f.write('\t\t\t\t<paramPoly3 pRange="arcLength" aU="' + aU + '" bU="' + bU + '" cU="' + cU + '" dU="' + dU + '" aV="' + aV + '" bV="' + bV + '" cV="' + cV + '" dV="' + dV +'"/>\n')
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
