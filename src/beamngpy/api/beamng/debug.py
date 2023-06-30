from __future__ import annotations

from typing import List

from beamngpy.logging import create_warning
from beamngpy.misc.colors import coerce_color
from beamngpy.types import Color, Float3, Float4, StrDict

from .base import Api


class DebugApi(Api):
    """
    An API for drawing debug graphical objects in the simulator.

    Args:
        beamng: An instance of the simulator.
    """

    def add_spheres(self, coordinates: List[Float3], radii: List[float],
                    rgba_colors: List[Color] | Color, cling: bool = False, offset: float = 0.) -> List[int]:
        """
        Adds graphical debug spheres to the simulator at positions specified by the
        **coordinates** argument.

        The arguments **coordinates**, **radii** and **rgba_colors** have to have the
        same length, which is the number of the debug spheres added.

        Args:
            coordinates: List of ``(x, y, z)`` coordinates of the debug spheres.
            radii: List of radii of the debug spheres in meters.
            rgba_colors: Either a single color or list of colors of the debug spheres,
                         in the format of ``(R, G, B, A)``. An ``A`` of 1.0 means full visibility,
                         0.0 means full transparency. Can also be instances of any type that the
                         :func:`coerce_color <beamngpy.misc.colors.coerce_color>` function accepts.
            cling: Whether or not to align the ``z`` coordinate of the spheres to the ground.
            offset: The z-axis offset of the sphere coordinates. Can only be used together with
                    ``cling=True`` to spawn spheres an exact amount above the ground.

        Returns:
            List of string IDs of the debug spheres added. This list can be passed to the
            :func:`remove_spheres` function.
        """
        if offset != 0. and not cling:
            create_warning('The `offset` argument is ignored when `cling` is set to False.')
        data: StrDict = dict(type='AddDebugSpheres')
        if not isinstance(rgba_colors, list):
            rgba_colors = [rgba_colors] * len(coordinates)

        assert len(coordinates) == len(radii) == len(rgba_colors)
        data['coordinates'] = coordinates
        data['radii'] = radii
        data['colors'] = [coerce_color(c, alpha=1.0) for c in rgba_colors]
        data['cling'] = cling
        data['offset'] = offset
        resp = self._send(data).recv('DebugSphereAdded')
        return [int(s) for s in resp['sphereIDs']]

    def remove_spheres(self, sphere_ids: List[int]) -> None:
        """
        Removes the spheres with the IDs provided in the **sphere_ids** argument.

        Args:
            sphere_ids: A list of the integer IDs of the spheres to be deleted.
        """
        data: StrDict = dict(type='RemoveDebugObjects')
        data['objType'] = 'spheres'
        data['objIDs'] = sphere_ids
        self._send(data).ack('DebugObjectsRemoved')

    def add_polyline(self, coordinates: List[Float3], rgba_color: Color,
                     cling: bool = False, offset: float = 0.) -> int:
        """
        Adds graphical debug polyline to the simulator with points at positions specified by the
        **coordinates** argument.

        The arguments **coordinates**, **radii** and **rgba_colors** have to have the
        same length, which is the number of the debug spheres added.

        Args:
            coordinates: List of ``(x, y, z)`` coordinates of the debug spheres.
            rgba_color: A single color of the points of the debug polyline, in the format of ``(R, G, B, A)``.
                        An ``A`` of 1.0 means full visibility, 0.0 means full transparency. Can also be instance
                        of any type that the :func:`coerce_color <beamngpy.misc.colors.coerce_color>` function accepts.
            cling: Whether or not to align the ``z`` coordinate of the spheres to the ground.
            offset: The z-axis offset of the sphere coordinates. Can only be used together with
                    ``cling=True`` to spawn spheres an exact amount above the ground.

        Returns:
            An integer ID of the debug polyline added. This ID can be passed to the :func:`remove_polyline` function.
        """
        if offset != 0. and not cling:
            create_warning('The `offset` argument is ignored when `cling` is set to False.')
        data: StrDict = dict(type='AddDebugPolyline')
        data['coordinates'] = coordinates
        data['color'] = coerce_color(rgba_color, alpha=1.0)
        data['cling'] = cling
        data['offset'] = offset
        resp = self._send(data).recv('DebugPolylineAdded')
        return int(resp['lineID'])

    def remove_polyline(self, line_id: int) -> None:
        """
        Removes the polyline with the ID provided in the **line_id** argument.

        Args:
            line_id: An integer ID of the polyline to be deleted.
        """
        data: StrDict = dict(type='RemoveDebugObjects')
        data['objType'] = 'polylines'
        data['objIDs'] = [line_id]
        self._send(data).ack('DebugObjectsRemoved')

    def add_cylinder(self, circle_positions, radius, rgba_color):
        data: StrDict = dict(type='AddDebugCylinder')
        data['circlePositions'] = circle_positions
        data['radius'] = radius
        data['color'] = rgba_color
        resp = self._send(data).recv('DebugCylinderAdded')
        return resp['cylinderID']

    def remove_cylinder(self, cylinder_id):
        data: StrDict = dict(type='RemoveDebugObjects')
        data['objType'] = 'cylinders'
        data['objIDs'] = [cylinder_id]
        self._send(data).ack('DebugObjectsRemoved')

    def add_triangle(self, vertices, rgba_color, cling=False, offset=0):
        data: StrDict = dict(type='AddDebugTriangle')
        data['vertices'] = vertices
        data['color'] = rgba_color
        data['cling'] = cling
        data['offset'] = offset
        resp = self._send(data).recv('DebugTriangleAdded')
        return resp['triangleID']

    def remove_triangle(self, triangle_id):
        data: StrDict = dict(type='RemoveDebugObjects')
        data['objType'] = 'triangles'
        data['objIDs'] = [triangle_id]
        self._send(data).ack('DebugObjectsRemoved')

    def add_rectangle(self, vertices, rgba_color, cling=False, offset=0):
        data: StrDict = dict(type='AddDebugRectangle')
        data['vertices'] = vertices
        data['color'] = rgba_color
        data['cling'] = cling
        data['offset'] = offset
        resp = self._send(data).recv('DebugRectangleAdded')
        return resp['rectangleID']

    def remove_rectangle(self, rectangle_id: str):
        data: StrDict = dict(type='RemoveDebugObjects')
        data['objType'] = 'rectangles'
        data['objIDs'] = [rectangle_id]
        return self._send(data).ack('DebugObjectsRemoved')

    def add_text(self, origin, content, rgba_color, cling=False, offset=0):
        data: StrDict = dict(type='AddDebugText')
        data['origin'] = origin
        data['content'] = content
        data['color'] = rgba_color
        data['cling'] = cling
        data['offset'] = offset
        resp = self._send(data).recv('DebugTextAdded')
        return resp['textID']

    def remove_text(self, text_id: str):
        data: StrDict = dict(type='RemoveDebugObjects')
        data['objType'] = 'text'
        data['objIDs'] = [text_id]
        return self._send(data).ack('DebugObjectsRemoved')

    def add_square_prism(self, end_points, end_point_dims, rgba_color):
        data = dict(type='AddDebugSquarePrism')
        data['endPoints'] = end_points
        data['dims'] = end_point_dims
        data['color'] = rgba_color
        resp = self._send(data).recv('DebugSquarePrismAdded')
        return resp['prismID']

    def remove_square_prism(self, prism_id: str):
        data: StrDict = dict(type='RemoveDebugObjects')
        data['objType'] = 'squarePrisms'
        data['objIDs'] = [prism_id]
        return self._send(data).ack('DebugObjectsRemoved')
