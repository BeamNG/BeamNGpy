from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any

from beamngpy.logging import LOGGER_ID, BNGError
from beamngpy.types import StrDict

from .communication_utils import send_sensor_request, set_sensor

import math
from datetime import datetime

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import collections  as mc

import xml.etree.ElementTree as ET

import seaborn as sns

sns.set()  # Let seaborn apply better styling to all matplotlib graphs

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy

__all__ = ['Road_Graph']

class Connection:
    def __init__(self, connection_type, id, contact_point):
        self.connection_type = connection_type
        self.id = id
        self.contact_point = contact_point

class Road:
    def __init__(self, start, end, predecessor, successor, start_x, start_y, length, start_elevation, linear_elevation, start_width, linear_width, hdg, junction, contact_point):
        self.start = start
        self.end = end
        self.predecessor = predecessor
        self.successor = successor
        self.start_x = start_x
        self.start_y = start_y
        self.length = length
        self.start_elevation = start_elevation
        self.linear_elevation = linear_elevation
        self.start_width = start_width
        self.linear_width = linear_width
        self.hdg = hdg
        self.junction = junction
        self.contact_point = contact_point

class Junction:
    def __init__(self, id, connection_roads, is_end_point):
        self.id = id
        self.connection_roads = connection_roads
        self.is_end_point = is_end_point



class Road_Graph:
    """
    A class for retrieving the road graph data from the simulator.
    Args:
        bng: The BeamNGpy instance, with which to communicate to the simulation.
    """

    def collection_does_not_contain_segment(self, collection, segment):
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
                            if self.collection_does_not_contain_segment(collection, current_path):
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
                            print("WARNING!!  BOTH CHILDREN OF NODE ARE IN LIST", self.coords[current_path[-1]])
                            if self.collection_does_not_contain_segment(collection, current_path):
                                collection[ctr] = current_path
                                ctr = ctr + 1
                            break

        # We want to keep the minimum length of a segment to 3 (not 2, which it currently is) so we split small segments in two.
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

    def export_xodr(self, name):

        # Compute all the individual path segments from the loaded map.
        path_segments = self.compute_path_segments()

        # Compute the uniquely-identifiable roads (between two nodes).
        roads = []
        for i in range(len(path_segments)):
            seg = path_segments[i]
            for j in range(1, len(seg)):
                start = seg[j - 1]
                end = seg[j]
                junction = None
                if j == 1:
                    junction = seg[0]
                elif j == len(seg) - 2:
                    junction = seg[-1]
                predecessor = None
                successor = None
                contact_point = None
                if j > 1:
                    predecessor = i - 1
                    contact_point = 'start'
                if j < len(seg) - 1:
                    successor = i + 1
                    contact_point = 'end'
                start_x = self.coords[seg[j - 1]][0]
                start_y = self.coords[seg[j - 1]][1]
                start_z = self.coords[seg[j - 1]][2]
                end_x = self.coords[seg[j]][0]
                end_y = self.coords[seg[j]][1]
                end_z = self.coords[seg[j]][2]
                dx = end_x - start_x
                dy = end_y - start_y
                length = math.sqrt(dx * dx + dy * dy)                   # road length (only use x and y - elevation is separate).
                linear_elevation = (end_z - start_z) / length           # linear elevation rate across road. TODO: maybe just end-start, no division here. check in .xodr !!!
                hdg = math.atan2(dy, dx)                                # heading (rad).
                start_width = self.widths[seg[j - 1]]
                end_width = self.widths[seg[j]]
                linear_width = (end_width - start_width) / length       # linear width rate across road. TODO: maybe just end-start, no division here. check in .xodr !!!
                roads.append(Road(start, end, predecessor, successor, start_x, start_y, length, start_z, linear_elevation, start_width, linear_width, hdg, junction, contact_point))

        # Create a map between junction key names and unique Id numbers.
        junction_map = {}
        ctr = 0
        for i in range(len(path_segments)):
            seg = path_segments[i]
            a = seg[0]
            if a not in junction_map:
                junction_map[a] = ctr
                ctr = ctr + 1
            b = seg[-1]
            if b not in junction_map:
                junction_map[b] = ctr
                ctr = ctr + 1

        # Create a list of uniquely-identifiable junctions.
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
                f.write('\t\t\t\t<line/>\n')
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

            #for i in range(len(junctions)):
            #    jct = junctions[i]
            #    f.write('\t<junction name="" id="' + str(jct.id) + '" type="default">\n')
            #    ctr = 0
            #    for j in range(len(jct.connection_roads)):
            #        a = jct.connection_roads[j]
            #        for k in range(len(jct.connection_roads)):
            #            b = jct.connection_roads[k]
            #            f.write('\t\t<connection id="' + str(ctr) + '" incomingRoad="' + str(a.id) + '" connectingRoad="' + str(b.id) + '" contactPoint="' + str(a.contact_point) + '">\n')
            #            if a.contact_point == 'start':
            #                f.write('\t\t\t<laneLink from="-1" to="1"/>\n')
            #            else:
            #                f.write('\t\t\t<laneLink from="1" to="-1"/>\n')
            #            f.write('\t\t</connection>\n')
            #            ctr = ctr + 1
            #    f.write('\t</junction>\n')

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
