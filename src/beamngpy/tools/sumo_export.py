from __future__ import annotations

from beamngpy.tools.navigraph_data import NavigraphData

__all__ = ["SumoExporter"]

NUM_LANES = "2"  # The number of lanes to use for roads in Sumo's roads.
PRIORITY = "2"  # The priority value to use, for all roads.
SPEED = "14.0"  # The speed value to use, for all roads.


class SumoExporter:

    @staticmethod
    def export(name, bng):
        """
        Exports the road network data to Sumo (.nod.xml and .edg.xml) format.
        The export contains all road sections, some basic lane data, and some junction connectivity data.
        This function will generate both .xml files required to generate the Sumo road network. The user should then type the following into the command prompt:
        `netconvert --node-files=<NAME>.nod.xml --edge-files=<NAME>.edg.xml -o converted.net.xml`
        which will then generate the final road network, which can be loaded with the sumo applications.

        Args:
            name: the filename prefix, by which to save the sumo road network (the .nod.xml and .edg.xml extensions will be appended to the end of this name).
            bng: The BeamNG instance.
        """

        # Get the navigraph data.
        navigraph_data = NavigraphData(bng)

        # Compute all the individual path segments from the loaded map.
        path_segments = navigraph_data.compute_path_segments()

        # Compute the list of unique nodes from the navigraph data. The keys are retained from the navigraph, and will be used in the sumo files.
        nodes = []
        for k, v in navigraph_data.coords3d.items():
            nodes.append([k, v.x, v.y, v.z])

        # Compute the list of unique edges, from the path segments. Each line segment from each path becomes a unique edge.
        edges = []
        for _, path in path_segments.items():
            for i in range(len(path) - 1):
                edges.append([path[i], path[i + 1]])

        # Write the node data file ('<NAME>.nod.xml').
        node_file_name = name + ".nod.xml"
        with open(node_file_name, "w") as f:
            f.write("<nodes>\n")
            for n in nodes:
                f.write(
                    '\t<node id="'
                    + n[0]
                    + '" x="'
                    + str(n[1])
                    + '" y="'
                    + str(n[2])
                    + '" z="'
                    + str(n[3])
                    + '"/>\n'
                )
            f.write("</nodes>\n")

        # Write the edge data file ('<NAME>.edg.xml').
        edge_file_name = name + ".edg.xml"
        ctr = 1
        with open(edge_file_name, "w") as f:
            f.write("<edges>\n")
            for e in edges:
                # First, we create an edge from A to B.
                f.write(
                    '\t<edge id="'
                    + ("e" + str(ctr))
                    + '" from="'
                    + str(e[0])
                    + '" to="'
                    + str(e[1])
                    + '" priority="'
                    + PRIORITY
                    + '" numLanes="'
                    + NUM_LANES
                    + '" speed="'
                    + SPEED
                    + '"/>\n'
                )
                ctr = ctr + 1

                # Then, we create an edge from B to A.  This is how to produce bi-directional roads.  Silence this if one-way is preferred.
                f.write(
                    '\t<edge id="'
                    + ("e" + str(ctr))
                    + '" from="'
                    + str(e[1])
                    + '" to="'
                    + str(e[0])
                    + '" priority="'
                    + PRIORITY
                    + '" numLanes="'
                    + NUM_LANES
                    + '" speed="'
                    + SPEED
                    + '"/>\n'
                )
                ctr = ctr + 1
            f.write("</edges>\n")
