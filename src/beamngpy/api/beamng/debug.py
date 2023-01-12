from __future__ import annotations

from typing import List

from beamngpy.types import Float3, Float4, StrDict

from .base import Api


class DebugApi(Api):
    """
    An API for drawing debug objects in the simulator.

    Args:
        beamng: An instance of the simulator.
    """

    def add_spheres(self, coordinates: List[Float3], radii: List[float],
                    rgba_colors: List[Float4], cling: bool = False, offset: float = 0):
        data: StrDict = dict(type='AddDebugSpheres')
        assert len(coordinates) == len(radii) == len(rgba_colors)
        data['coordinates'] = coordinates
        data['radii'] = radii
        data['colors'] = rgba_colors
        data['cling'] = cling
        data['offset'] = offset
        resp = self._send(data).recv('DebugSphereAdded')
        return resp['sphereIDs']

    def remove_spheres(self, sphere_ids: List[str]):
        data: StrDict = dict(type='RemoveDebugObjects')
        data['objType'] = 'spheres'
        data['objIDs'] = sphere_ids
        self._send(data).ack('DebugObjectsRemoved')

    def add_polyline(self, coordinates: List[Float3], rgba_color: List[Float4],
                     cling: bool = False, offset: float = 0):
        data: StrDict = dict(type='AddDebugPolyline')
        data['coordinates'] = coordinates
        data['color'] = rgba_color
        data['cling'] = cling
        data['offset'] = offset
        resp = self._send(data).recv('DebugPolylineAdded')
        return resp['lineID']

    def remove_polyline(self, line_id: str):
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
