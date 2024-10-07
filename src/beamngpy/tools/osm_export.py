from __future__ import annotations

from datetime import datetime

from beamngpy import vec3
from beamngpy.tools.navigraph_data import NavigraphData

__all__ = ["OpenStreetMapExporter"]


class OpenStreetMapExporter:

    @staticmethod
    def export(name, bng):
        """
        Exports the road network data to OpenStreetMap (.osm) format.
        The export contains all road sections, some basic lane data, and some junction connectivity data.

        Args:
            name: The path/filename by which to save the .osm file.
            bng: The BeamNG instance.
        """

        # Get the navigraph data.
        navigraph_data = NavigraphData(bng)

        # Compute all the individual path segments from the loaded map.
        path_segments = navigraph_data.compute_path_segments()

        # Create the nodes data: A unique list of nodes, a map from graph keys to unique node id, and bounds info.
        scale_factor = (
            1.0 / 1e7
        )  # to convert metres into reasonable lattitude/longitude values.
        nodes = []
        keys_to_node_map = {}
        minlat = 1e99
        minlon = 1e99
        maxlat = -1e99
        maxlon = -1e99
        ctr = 0
        for k, v in navigraph_data.coords3d.items():
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
        file_name = name + ".osm"
        date_time = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        with open(file_name, "w") as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            f.write('<osm version="0.6" generator="BeamNGPy">\n')
            f.write(
                '\t<bounds minlat="'
                + str(minlat)
                + '" minlon="'
                + str(minlon)
                + '" maxlat="'
                + str(maxlat)
                + '" maxlon="'
                + str(maxlon)
                + '"/>\n'
            )
            for i in range(len(nodes)):
                nodeId = i + 1
                lat = str(nodes[i][0])
                lon = str(nodes[i][1])
                ele = str(nodes[i][2])
                f.write(
                    '\t<node id="'
                    + str(nodeId)
                    + '" lat="'
                    + lat
                    + '" lon="'
                    + lon
                    + '" ele="'
                    + ele
                    + '" user="BeamNG" uid="1" visible="true" version="1" changeset="1" timestamp="'
                    + date_time
                    + '"/>\n'
                )
            for i in range(len(ways)):
                wayId = i + 1  # the OpenStreetMap Id numbers start at 1 not 0.
                f.write(
                    '\t<way id="'
                    + str(wayId)
                    + '" user="BeamNG" uid="1" visible="true" version="1" changeset="1">\n'
                )
                seg = ways[i]
                for j in range(len(seg)):
                    nodeId = seg[j] + 1
                    f.write('\t\t<nd ref="' + str(nodeId) + '"/>\n')
                f.write("\t</way>\n")
            f.write("</osm>\n")
