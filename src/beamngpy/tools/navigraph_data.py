from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any

import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib import collections as mc

from beamngpy import vec3
from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID
from beamngpy.types import StrDict

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy


class NavigraphData(CommBase):

    def __init__(self, bng: BeamNGpy):
        """
        Fetches the raw navigraph data from the simulator.

        Args:
            bng: The BeamNG instance.
        """
        super().__init__(bng, None)

        self.logger = getLogger(f"{LOGGER_ID}.Road_Graph")
        self.logger.setLevel(DEBUG)

        # Get the road graph data for the current map.
        raw_data = self.send_recv_ge("GetRoadGraph")["data"]
        for key in raw_data:  # fix the types if no roads fround
            raw_data[key] = dict(raw_data[key])
        self.graph = raw_data["graph"]
        self.coords3d = self._to_vec3(raw_data["coords"])
        self.coords2d = self.get_2d_coords()
        self.widths = raw_data["widths"]
        self.normals = self._to_vec3(raw_data["normals"])
        self._cached_tangents = {}

        self.logger.debug("Road_Graph - data retrieved.")

    def get_2d_coords(self):
        coords_2d = {}
        for k, v in self.coords3d.items():
            coords_2d[k] = vec3(v.x, v.y, 0.0)
        return coords_2d

    def _to_vec3(self, d):
        output = {}
        for k, v in d.items():
            output[k] = vec3(v[0], v[1], v[2])
        return output

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
        graph = self.graph
        for head_key in graph.keys():
            successors = graph[head_key].keys()
            if len(successors) != 2:
                for child_key in successors:
                    current_path = []
                    current_path.append(head_key)
                    next_key = child_key
                    while True:
                        current_path.append(next_key)
                        next_successors = graph[next_key].keys()
                        if len(next_successors) != 2:
                            if self._collection_does_not_contain_segment(
                                collection, current_path
                            ):
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
                            if self._collection_does_not_contain_segment(
                                collection, current_path
                            ):
                                collection[ctr] = current_path
                                ctr = ctr + 1
                            break
        return collection

    @staticmethod
    def plot_path_segments(path_segments, coords3d):
        """
        Debugging tool to display the individual path segments of the BeamNG navigraph.
        Each junction node is highlighted in red.  Each intermediate path node is highlighted in blue.

        Args:
            path_segments: The collection of individual 'path segments' to plot.
            coords3d: The 3d coordinates data from the navgraph (comes from the exporter file).
        """
        sns.set_theme()  # Let seaborn apply better styling to all matplotlib graphs

        fig, ax = plt.subplots(figsize=(15, 15))
        px = []
        py = []
        lines = []
        line_colors = []
        node_colors = []
        for i in range(len(path_segments)):
            seg = path_segments[i]
            px.append(coords3d[seg[0]][0])
            py.append(coords3d[seg[0]][1])
            node_colors.append((1.0, 0.0, 0.0, 1.0))
            for j in range(1, len(seg)):
                px.append(coords3d[seg[j]][0])
                py.append(coords3d[seg[j]][1])
                node_colors.append((0.0, 0.0, 1.0, 1.0))
                p1 = coords3d[seg[j - 1]]
                p2 = coords3d[seg[j]]
                lines.append([(p1[0], p1[1]), (p2[0], p2[1])])
                line_colors.append((0.3, 0.3, 0.3, 0.5))
            node_colors[-1] = (1.0, 0.0, 0.0, 1.0)

        ax.add_collection(mc.LineCollection(lines, colors=line_colors, linewidths=0.5))
        ax.scatter(px, py, s=3.0, c=node_colors, cmap=matplotlib.cm.viridis)
        plt.show()
