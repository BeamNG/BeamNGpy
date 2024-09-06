from __future__ import annotations

import math
from datetime import datetime

import numpy as np

from beamngpy import vec3
from beamngpy.tools.navigraph_data import NavigraphData

__all__ = ["OpenDriveExporter"]

CACHED_TANGENTS = {}


class ExplicitCubic:
    """
    A class for representing explicit cubic polynomials of the form: [ u(p) := a + b*p + c*p^2 + d*p^3 ].
    """

    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d


class ParametricCubic:
    """
    A class for representing parametric cubic polynomials of the form:
    [ u(x) := Bu*x + Cu^2 + Du^3; v(x) := Bv*x + Cv^2 + Dv^3 ].
    """

    def __init__(self, Au, Bu, Cu, Du, Av, Bv, Cv, Dv):
        self.Au = Au
        self.Bu = Bu
        self.Cu = Cu
        self.Du = Du
        self.Av = Av
        self.Bv = Bv
        self.Cv = Cv
        self.Dv = Dv


class Road:
    """
    A container for storing single sections of roads, which can be represented by a single geometric primitive.
    """

    def __init__(
        self,
        start,
        end,
        p1,
        length,
        ref_line_cubic,
        elev_cubic,
        width_cubic,
        super_elev_cubic,
        predecessor=None,
        successor=None,
        junction=None,
        contact_point=None,
    ):
        self.start = start
        self.end = end
        self.p1 = p1
        self.length = length
        self.ref_line_cubic = ref_line_cubic
        self.elev_cubic = elev_cubic
        self.width_cubic = width_cubic
        self.super_elev_cubic = super_elev_cubic
        self.predecessor = predecessor
        self.successor = successor
        self.junction = junction
        self.contact_point = contact_point

    def update_connection_data(self, predecessor, successor, junction, contact_point):
        """
        Updates the road/junction connectivity data associated with this Road.

        Args:
            predecessor: The graph key of the road which preceeds this road in the path segment, if it exists.
            successor: The graph key of the road which succeeds this road in the path segment, if it exists.
            junction: The Id of the junction, if the road is connected to a junction. Should be -1 otherwise.
            contact_point: Either 'start' or 'end'.  This is which end of the road is connected to the junction, if there is a junction.
        """
        self.predecessor = predecessor
        self.successor = successor
        self.junction = junction
        self.contact_point = contact_point


class Junction:
    """
    A class for storing road network junction information.

    Args:
        id: The unique Id of this junction.
        connection_roads: The collection of roads which connect to this junction.
    """

    def __init__(self, id, connection_roads):
        self.id = id
        self.connection_roads = connection_roads


class ConnectionRoad:
    """
    A class for storing connectivity information between a road and a junction.

    Args:
        id: The unique Id of this road, in the roads collection.
        contact_point: Either 'start' or 'end'. This is which part of the road connects to the junction.
    """

    def __init__(self, id, contact_point):
        self.id = id
        self.contact_point = contact_point


class OpenDriveExporter:
    """
    A class for exporting BeamNG road network data to OpenDrive (.xodr) format.
    """

    @staticmethod
    def _graph_key_to_junction_map(graph, path_segments):
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
            # If this node is a dead-end, we do not include it as a junction.
            if len(graph[key1].keys()) < 2:
                continue
            # The first node in a path segment is a junction node. Store it if we have not already found it.
            if key1 not in junction_map:
                junction_map[key1] = ctr
                ctr = ctr + 1
            key2 = seg[-1]
            # If this node is a dead-end, we do not include it as a junction.
            if len(graph[key2].keys()) < 2:
                continue
            # The last node in a path segment is also a junction node. Store it if we have not already found it.
            if key2 not in junction_map:
                junction_map[key2] = ctr
                ctr = ctr + 1
        return junction_map

    @staticmethod
    def _compute_tangents(pn0, pn1, pn2, pn3):
        d1, d2, d3 = (
            max(math.sqrt(pn0.distance(pn1)), 1e-12),
            math.sqrt(pn1.distance(pn2)),
            max(math.sqrt(pn2.distance(pn3)), 1e-12),
        )
        m = (pn1 - pn0) / d1 + (pn0 - pn2) / (d1 + d2)
        n = (pn1 - pn3) / (d2 + d3) + (pn3 - pn2) / d3
        t1 = d2 * m + pn2 - pn1
        if t1.length() < 1e-5:
            t1 = (pn2 - pn1) * 0.5
        t2 = d2 * n + pn2 - pn1
        if t2.length() < 1e-5:
            t2 = (pn2 - pn1) * 0.5
        return [t1, t2]

    @staticmethod
    def _fit_cubic(p0, p1, p2, p3, length):
        [tang1, tang2] = OpenDriveExporter._compute_tangents(
            p0 - p1, vec3(0.0, 0.0, 0.0), p2 - p1, p3 - p1
        )
        dz = p2.z - p1.z
        length_sq = length * length
        return ExplicitCubic(
            p1.z,
            tang1.z / length,
            ((-2.0 * tang1.z) - tang2.z + (3.0 * dz)) / length_sq,
            (tang1.z + tang2.z - (2.0 * dz)) / (length_sq * length),
        )

    @staticmethod
    def _find_trunk_road_tangent(key, std_tangent):
        length = std_tangent.length()
        if key in CACHED_TANGENTS:
            std_tan_norm = std_tangent.normalize()
            max_so_far = -1.0
            max_idx = 0
            best_dot = 0.0
            is_found = False
            for t in range(len(CACHED_TANGENTS[key])):
                test_tan_norm = CACHED_TANGENTS[key][t].normalize()
                short_angle = std_tan_norm.dot(test_tan_norm)
                if abs(short_angle) > max(0.707, max_so_far):
                    max_so_far = abs(short_angle)
                    max_idx = t
                    best_dot = short_angle
                    is_found = True
            if is_found == True:
                if best_dot < 0.0:
                    aaa = -1.0 * CACHED_TANGENTS[key][max_idx]
                    return aaa.normalize() * length
                bbb = CACHED_TANGENTS[key][max_idx]
                return bbb.normalize() * length
            else:
                CACHED_TANGENTS[key].append(std_tangent)
        else:
            CACHED_TANGENTS[key] = [std_tangent]
        return std_tangent

    @staticmethod
    def _compute_roll_angles(graph, seg, i0, i1, i2, i3):
        t0n = graph.coords3d[seg[i1]] - graph.coords3d[seg[i0]]
        t1n = graph.coords3d[seg[i2]] - graph.coords3d[seg[i0]]
        t2n = graph.coords3d[seg[i3]] - graph.coords3d[seg[i1]]
        t3n = graph.coords3d[seg[i3]] - graph.coords3d[seg[i2]]
        if t0n.length() < 1e-5:
            t0n = graph.coords3d[seg[i2]] - graph.coords3d[seg[i1]]
        if t1n.length() < 1e-5:
            t1n = graph.coords3d[seg[i2]] - graph.coords3d[seg[i1]]
        if t2n.length() < 1e-5:
            t2n = graph.coords3d[seg[i2]] - graph.coords3d[seg[i1]]
        if t3n.length() < 1e-5:
            t3n = graph.coords3d[seg[i2]] - graph.coords3d[seg[i1]]
        t0n = t0n.normalize()
        t1n = t1n.normalize()
        t2n = t2n.normalize()
        t3n = t3n.normalize()

        n0 = graph.normals[seg[i0]].normalize()
        n1 = graph.normals[seg[i1]].normalize()
        n2 = graph.normals[seg[i2]].normalize()
        n3 = graph.normals[seg[i3]].normalize()

        proj0 = n0 - (n0.dot(t0n)) * t0n
        proj1 = n1 - (n1.dot(t1n)) * t1n
        proj2 = n2 - (n2.dot(t2n)) * t2n
        proj3 = n3 - (n3.dot(t3n)) * t3n

        vertical = vec3(0.0, 0.0, 1.0)
        ra0 = math.acos(max(min(proj0.dot(vertical), 1.0), -1.0))
        ra1 = math.acos(max(min(proj1.dot(vertical), 1.0), -1.0))
        ra2 = math.acos(max(min(proj2.dot(vertical), 1.0), -1.0))
        ra3 = math.acos(max(min(proj3.dot(vertical), 1.0), -1.0))
        if proj0.x < 0.0:
            ra0 = ra0 * -1.0
        if proj1.x < 0.0:
            ra1 = ra1 * -1.0
        if proj2.x < 0.0:
            ra2 = ra2 * -1.0
        if proj3.x < 0.0:
            ra3 = ra3 * -1.0

        return [ra0, ra1, ra2, ra3]

    @staticmethod
    def compute_roads_and_junctions(navigraph_data, path_segments):
        """
        Computes a collection of individual road sections and junctions, both indexed by a unique Id.
        This function produces all the relevant data ready to be exported to OpenDrive (.xodr) format.
        """

        # Compute all the individual path segments from the loaded map. Sort them in order of widths, descending.
        unsorted_path_segments = []
        for i in range(len(path_segments)):
            unsorted_path_segments.append(path_segments[i])
        widths = []
        for i in range(len(unsorted_path_segments)):
            seg = unsorted_path_segments[i]
            avg_width = 0.0
            for j in range(len(seg)):
                avg_width += navigraph_data.widths[seg[j]]
            widths.append(avg_width / len(seg))
        sort_index = np.argsort(widths)[::-1]

        # Compute a uniquely-identifiable list of roads (between two nodes).
        CACHED_TANGENTS = {}
        roads = []
        for i in range(len(path_segments)):
            seg = path_segments[sort_index[i]]
            for i1 in range(len(seg) - 1):
                # Compute the reference line cubic equations (parametric).
                i0, i2, i3 = max(i1 - 1, 0), i1 + 1, min(i1 + 2, len(seg) - 1)
                seg0, seg1, seg2, seg3 = seg[i0], seg[i1], seg[i2], seg[i3]
                p1_2d = navigraph_data.coords2d[seg1]
                pn0_2d, pn1_2d, pn2_2d, pn3_2d = (
                    navigraph_data.coords2d[seg0] - p1_2d,
                    vec3(0.0, 0.0, 0.0),
                    navigraph_data.coords2d[seg2] - p1_2d,
                    navigraph_data.coords2d[seg3] - p1_2d,
                )
                geodesic_length_2d = pn1_2d.distance(pn2_2d)
                [tang1, tang2] = OpenDriveExporter._compute_tangents(
                    pn0_2d, pn1_2d, pn2_2d, pn3_2d
                )
                if i1 == 0:
                    tang1 = OpenDriveExporter._find_trunk_road_tangent(seg1, tang1)
                if i1 == len(seg) - 2:
                    tang2 = OpenDriveExporter._find_trunk_road_tangent(seg2, tang2)
                coeff_C, coeff_D = (-2.0 * tang1) - tang2 + (
                    3.0 * pn2_2d
                ), tang1 + tang2 - (2.0 * pn2_2d)
                ref_line_cubic = ParametricCubic(
                    pn1_2d.x,
                    tang1.x,
                    coeff_C.x,
                    coeff_D.x,
                    pn1_2d.y,
                    tang1.y,
                    coeff_C.y,
                    coeff_D.y,
                )

                # Compute the elevation cubic equation (explicit).
                p0_3d, p1_3d, p2_3d, p3_3d = (
                    navigraph_data.coords3d[seg0],
                    navigraph_data.coords3d[seg1],
                    navigraph_data.coords3d[seg2],
                    navigraph_data.coords3d[seg3],
                )
                elev_cubic = OpenDriveExporter._fit_cubic(
                    p0_3d, p1_3d, p2_3d, p3_3d, geodesic_length_2d
                )

                # Compute the width cubic equation (explicit).
                w0, w1, w2, w3 = (
                    navigraph_data.widths[seg0],
                    navigraph_data.widths[seg1],
                    navigraph_data.widths[seg2],
                    navigraph_data.widths[seg3],
                )
                width_cubic = OpenDriveExporter._fit_cubic(
                    vec3(p0_3d.x, p0_3d.y, w0),
                    vec3(p1_3d.x, p1_3d.y, w1),
                    vec3(p2_3d.x, p2_3d.y, w2),
                    vec3(p3_3d.x, p3_3d.y, w3),
                    geodesic_length_2d,
                )

                # Compute the super-elevation cubic equation (explicit).
                [roll0, roll1, roll2, roll3] = OpenDriveExporter._compute_roll_angles(
                    navigraph_data, seg, i0, i1, i2, i3
                )
                roll0 = math.pi / 4
                roll1 = math.pi / 4
                roll2 = math.pi / 4
                roll3 = math.pi / 4
                super_elev_cubic = OpenDriveExporter._fit_cubic(
                    vec3(p0_3d.x, p0_3d.y, roll0),
                    vec3(p1_3d.x, p1_3d.y, roll1),
                    vec3(p2_3d.x, p2_3d.y, roll2),
                    vec3(p3_3d.x, p3_3d.y, roll3),
                    geodesic_length_2d,
                )

                # Create the road section.
                roads.append(
                    Road(
                        seg1,
                        seg2,
                        navigraph_data.coords3d[seg1],
                        geodesic_length_2d,
                        ref_line_cubic,
                        elev_cubic,
                        width_cubic,
                        super_elev_cubic,
                    )
                )

        # Create a map between junction key names and unique Id numbers.
        junction_map = OpenDriveExporter._graph_key_to_junction_map(
            navigraph_data.graph, path_segments
        )

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
                    contact_point = "start"
                if r.end == roads[j].start:
                    successor = j
                    contact_point = "end"
            if (
                r.start in junction_map
            ):  # Compute the junction and contact point data for this road, if they exist.
                junction = junction_map[r.start]
                contact_point = "start"
            elif r.end in junction_map:
                junction = junction_map[r.end]
                contact_point = "end"
            roads[i].update_connection_data(
                predecessor, successor, junction, contact_point
            )

        # Create a list of uniquely-identifiable junctions, containing all the relevant connection data.
        junctions = []
        for key, id in junction_map.items():
            connection_roads = []
            for i in range(len(roads)):
                r = roads[i]
                if key == r.start:
                    connection_roads.append(ConnectionRoad(i, "start"))
                if key == r.end:
                    connection_roads.append(ConnectionRoad(i, "end"))
            if len(connection_roads) > 0:
                junctions.append(Junction(id, connection_roads))

        return {"roads": roads, "junctions": junctions}

    @staticmethod
    def export(name, bng):
        """
        Exports the road network data to OpenDrive (.xodr) format.
        The export contains all road sections, some basic lane data, and some junction connectivity data.

        Args:
            name: The path/filename at which to save the .xodr file.
            bng: The BeamNG instance.
        """

        # Get the navigraph data.
        navigraph_data = NavigraphData(bng)

        # Compute all the individual path segments from the loaded map.
        path_segments = navigraph_data.compute_path_segments()

        # Get the road data.
        road_data = OpenDriveExporter.compute_roads_and_junctions(
            navigraph_data, path_segments
        )

        # Write the road network data to .xodr format (xml).
        date_time = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        file_name = name + ".xodr"
        with open(file_name, "w") as f:

            # .xodr file pre-amble.
            f.write('<?xml version="1.0" standalone="yes"?>\n')
            f.write("<OpenDRIVE>\n")
            f.write(
                '\t<header revMajor="1" revMinor="7" name="" version="1.00" date="'
                + date_time
                + '" north="0.0000000000000000e+00" south="0.0000000000000000e+00" east="0.0000000000000000e+00" west="0.0000000000000000e+00">\n'
            )
            f.write("\t</header>\n")

            # Write the road data, in order.
            for i in range(len(road_data["roads"])):
                r = road_data["roads"][i]

                # Road header data.
                f.write(
                    '\t<road rule="RHT" length="'
                    + str(r.length)
                    + '" id="'
                    + str(i)
                    + '" junction="'
                    + str(r.junction)
                    + '" >\n'
                )

                # Road connectivity data.
                f.write("\t\t<link>\n")
                if r.predecessor != "none":
                    f.write(
                        '\t\t\t<predecessor elementType="'
                        + "road"
                        + '" elementId="'
                        + str(r.predecessor)
                        + '" contactPoint="'
                        + str(r.contact_point)
                        + '" />\n'
                    )
                if r.successor != "none":
                    f.write(
                        '\t\t\t<successor elementType="'
                        + "road"
                        + '" elementId="'
                        + str(r.successor)
                        + '" contactPoint="'
                        + str(r.contact_point)
                        + '" />\n'
                    )
                f.write("\t\t</link>\n")

                # Geometry data.
                Au, Bu, Cu, Du = (
                    r.ref_line_cubic.Au,
                    r.ref_line_cubic.Bu,
                    r.ref_line_cubic.Cu,
                    r.ref_line_cubic.Du,
                )
                Av, Bv, Cv, Dv = (
                    r.ref_line_cubic.Av,
                    r.ref_line_cubic.Bv,
                    r.ref_line_cubic.Cv,
                    r.ref_line_cubic.Dv,
                )
                f.write(
                    '\t\t<type s="0.0000000000000000e+00" type="town" country="DE"/>\n'
                )
                f.write("\t\t<planView>\n")
                f.write(
                    '\t\t\t<geometry s="0.0000000000000000e+00" x="'
                    + str(r.p1.x)
                    + '" y="'
                    + str(r.p1.y)
                    + '" hdg="'
                    + str(0.0)
                    + '" length="'
                    + str(r.length)
                    + '">\n'
                )
                f.write(
                    '\t\t\t\t<paramPoly3 aU="'
                    + str(Au)
                    + '" bU="'
                    + str(Bu)
                    + '" cU="'
                    + str(Cu)
                    + '" dU="'
                    + str(Du)
                    + '" aV="'
                    + str(Av)
                    + '" bV="'
                    + str(Bv)
                    + '" cV="'
                    + str(Cv)
                    + '" dV="'
                    + str(Dv)
                    + '"/>\n'
                )
                f.write("\t\t\t</geometry>\n")
                f.write("\t\t</planView>\n")

                # Elevation data.
                Ae, Be, Ce, De = (
                    r.elev_cubic.a,
                    r.elev_cubic.b,
                    r.elev_cubic.c,
                    r.elev_cubic.d,
                )
                f.write("\t\t<elevationProfile>\n")
                f.write(
                    '\t\t\t<elevation s="0.0" a="'
                    + str(Ae)
                    + '" b="'
                    + str(Be)
                    + '" c="'
                    + str(Ce)
                    + '" d="'
                    + str(De)
                    + '"/>\n'
                )
                f.write("\t\t</elevationProfile>\n")

                # Super-elevation data (CURRENTLY NOT USED).
                Ase, Bse, Cse, Dse = (
                    r.super_elev_cubic.a,
                    r.super_elev_cubic.b,
                    r.super_elev_cubic.c,
                    r.super_elev_cubic.d,
                )
                f.write("\t\t<lateralProfile>\n")
                # f.write('\t\t\t<superelevation s="0.0" a="' + str(Ase) + '" b="' + str(Bse) + '" c="' + str(Cse) + '" d="' + str(Dse) + '"/>\n')
                f.write("\t\t</lateralProfile>\n")

                # Road lane data.
                Aw, Bw, Cw, Dw = (
                    r.width_cubic.a,
                    r.width_cubic.b,
                    r.width_cubic.c,
                    r.width_cubic.d,
                )
                f.write("\t\t<lanes>\n")
                f.write('\t\t\t<laneSection s="0.0000000000000000e+00">\n')
                f.write("\t\t\t\t<left>\n")
                f.write('\t\t\t\t\t<lane id="1" type="driving" level="false">\n')
                f.write("\t\t\t\t\t\t<link>\n")
                f.write("\t\t\t\t\t\t</link>\n")
                f.write(
                    '\t\t\t\t\t\t<width sOffset="0.0000000000000000e+00" a="'
                    + str(Aw)
                    + '" b="'
                    + str(Bw)
                    + '" c="'
                    + str(Cw)
                    + '" d="'
                    + str(Dw)
                    + '"/>\n'
                )
                f.write("\t\t\t\t\t</lane>\n")
                f.write("\t\t\t\t</left>\n")
                f.write("\t\t\t\t<center>\n")
                f.write('\t\t\t\t\t<lane id="0" type="driving" level="false">\n')
                f.write("\t\t\t\t\t\t<link>\n")
                f.write("\t\t\t\t\t\t</link>\n")
                f.write(
                    '\t\t\t\t\t\t<roadMark sOffset="0.0000000000000000e+00" type="broken" weight="standard" color="standard" width="1.2000000000000000e-01" laneChange="both" height="1.9999999552965164e-02">\n'
                )
                f.write(
                    '\t\t\t\t\t\t\t<type name="broken" width="1.2000000000000000e-01">\n'
                )
                f.write(
                    '\t\t\t\t\t\t\t\t<line length="3.0000000000000000e+00" space="6.0000000000000000e+00" tOffset="0.0000000000000000e+00" sOffset="0.0000000000000000e+00" rule="caution" width="1.2000000000000000e-01"/>\n'
                )
                f.write("\t\t\t\t\t\t\t</type>\n")
                f.write("\t\t\t\t\t\t</roadMark>\n")
                f.write("\t\t\t\t\t</lane>\n")
                f.write("\t\t\t\t</center>\n")
                f.write("\t\t\t\t<right>\n")
                f.write('\t\t\t\t\t<lane id="-1" type="driving" level="false">\n')
                f.write("\t\t\t\t\t\t<link>\n")
                f.write("\t\t\t\t\t\t</link>\n")
                f.write(
                    '\t\t\t\t\t\t<width sOffset="0.0000000000000000e+00" a="'
                    + str(Aw)
                    + '" b="'
                    + str(Bw)
                    + '" c="'
                    + str(Cw)
                    + '" d="'
                    + str(Dw)
                    + '"/>\n'
                )
                f.write("\t\t\t\t\t</lane>\n")
                f.write("\t\t\t\t</right>\n")
                f.write("\t\t\t</laneSection>\n")
                f.write("\t\t</lanes>\n")

                # TODO : WE DO NOT CURRENTLY MAKE USE OF THESE SECTIONS.
                f.write("\t\t<objects>\n")
                f.write("\t\t</objects>\n")
                f.write("\t\t<signals>\n")
                f.write("\t\t</signals>\n")
                f.write("\t\t<surface>\n")
                f.write("\t\t</surface>\n")

                f.write("\t</road>\n")

            # Write the junction data, in order.
            for i in range(len(road_data["junctions"])):
                jct = road_data["junctions"][i]
                f.write(
                    '\t<junction name="" id="' + str(jct.id) + '" type="default">\n'
                )
                ctr = 0
                for j in range(len(jct.connection_roads)):
                    ra = jct.connection_roads[j]
                    for k in range(len(jct.connection_roads)):
                        if j == k:
                            continue
                        rb = jct.connection_roads[k]
                        f.write(
                            '\t\t<connection id="'
                            + str(ctr)
                            + '" incomingRoad="'
                            + str(ra.id)
                            + '" connectingRoad="'
                            + str(rb.id)
                            + '" contactPoint="'
                            + str(ra.contact_point)
                            + '">\n'
                        )
                        if ra.contact_point == "start":
                            f.write('\t\t\t<laneLink from="-1" to="1"/>\n')
                        else:
                            f.write('\t\t\t<laneLink from="1" to="-1"/>\n')
                        f.write("\t\t</connection>\n")
                        ctr = ctr + 1
                f.write("\t</junction>\n")

            f.write("</OpenDRIVE>\n")
