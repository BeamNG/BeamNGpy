from __future__ import annotations

from typing import List

from beamngpy.logging import BNGValueError, create_warning
from beamngpy.misc.colors import coerce_color
from beamngpy.types import Color, Float2, Float3, StrDict

from .base import Api


class DebugApi(Api):
    """
    An API for drawing debug graphical objects in the simulator.

    Args:
        beamng: An instance of the simulator.
    """

    def add_spheres(
        self,
        coordinates: List[Float3],
        radii: List[float],
        rgba_colors: List[Color] | Color,
        cling: bool = False,
        offset: float = 0.0,
    ) -> List[int]:
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
                         :func:`coerce_color() <beamngpy.misc.colors.coerce_color>` function accepts.
            cling: Whether or not to align the ``z`` coordinate of the spheres to the ground.
            offset: The z-axis offset of the sphere coordinates. Can only be used together with
                    ``cling=True`` to spawn spheres an exact amount above the ground.

        Returns:
            List of string IDs of the debug spheres added. This list can be passed to the
            :func:`remove_spheres` function.
        """
        if offset != 0.0 and not cling:
            create_warning(
                "The `offset` argument is ignored when `cling` is set to False."
            )
        data: StrDict = dict(type="AddDebugSpheres")
        if not isinstance(rgba_colors, list):
            rgba_colors = [rgba_colors] * len(coordinates)

        assert len(coordinates) == len(radii) == len(rgba_colors)
        data["coordinates"] = coordinates
        data["radii"] = radii
        data["colors"] = [coerce_color(c, alpha=1.0) for c in rgba_colors]
        data["cling"] = cling
        data["offset"] = offset
        resp = self._send(data).recv("DebugSphereAdded")
        return [int(s) for s in resp["sphereIDs"]]

    def remove_spheres(self, sphere_ids: List[int]) -> None:
        """
        Removes the spheres with the IDs provided in the **sphere_ids** argument.

        Args:
            sphere_ids: A list of the integer IDs of the spheres to be deleted.
        """
        data: StrDict = dict(type="RemoveDebugObjects")
        data["objType"] = "spheres"
        data["objIDs"] = sphere_ids
        self._send(data).ack("DebugObjectsRemoved")

    def add_polyline(
        self,
        coordinates: List[Float3],
        rgba_color: Color,
        cling: bool = False,
        offset: float = 0.0,
    ) -> int:
        """
        Adds graphical debug polyline to the simulator with points at positions specified by the
        **coordinates** argument.

        The arguments **coordinates**, **radii** and **rgba_colors** have to have the
        same length, which is the number of the debug spheres added.

        Args:
            coordinates: List of ``(x, y, z)`` coordinates of the debug spheres.
            rgba_color: A single color of the points of the debug polyline, in the format of ``(R, G, B, A)``.
                        An ``A`` of 1.0 means full visibility, 0.0 means full transparency. Can also be instance
                        of any type that the :func:`coerce_color() <beamngpy.misc.colors.coerce_color>` function accepts.
            cling: Whether or not to align the ``z`` coordinate of the spheres to the ground.
            offset: The z-axis offset of the sphere coordinates. Can only be used together with
                    ``cling=True`` to spawn spheres an exact amount above the ground.

        Returns:
            An integer ID of the debug polyline added. This ID can be passed to the :func:`remove_polyline` function.
        """
        if offset != 0.0 and not cling:
            create_warning(
                "The `offset` argument is ignored when `cling` is set to False."
            )
        data: StrDict = dict(type="AddDebugPolyline")
        data["coordinates"] = coordinates
        data["color"] = coerce_color(rgba_color, alpha=1.0)
        data["cling"] = cling
        data["offset"] = offset
        resp = self._send(data).recv("DebugPolylineAdded")
        return int(resp["lineID"])

    def remove_polyline(self, line_id: int) -> None:
        """
        Removes the polyline with the ID provided in the **line_id** argument.

        Args:
            line_id: An integer ID of the polyline to be deleted.
        """
        data: StrDict = dict(type="RemoveDebugObjects")
        data["objType"] = "polylines"
        data["objIDs"] = [line_id]
        self._send(data).ack("DebugObjectsRemoved")

    def add_cylinder(
        self, circle_positions: List[Float3], radius: float, rgba_color: Color
    ) -> int:
        """
        Adds graphical debug cylinder to the simulator with bases at positions specified by the
        **circle_positions** argument.

        Args:
            circle_positions: List of two ``(x, y, z)`` coordinates of the circle centers.
            radius: The radius of the cylinder.
            rgba_color: A single color of the points of the debug cylinder, in the format of ``(R, G, B, A)``.
                        An ``A`` of 1.0 means full visibility, 0.0 means full transparency. Can also be instance
                        of any type that the :func:`coerce_color() <beamngpy.misc.colors.coerce_color>` function accepts.

        Returns:
            An integer ID of the debug cylinder added. This ID can be passed to the :func:`remove_cylinder` function.
        """
        if not len(circle_positions) == 2:
            raise BNGValueError("`circle_positions` needs to be a list of length 2!")

        data: StrDict = dict(type="AddDebugCylinder")
        data["circlePositions"] = circle_positions
        data["radius"] = radius
        data["color"] = coerce_color(rgba_color, alpha=1.0)
        resp = self._send(data).recv("DebugCylinderAdded")
        return int(resp["cylinderID"])

    def remove_cylinder(self, cylinder_id: int) -> None:
        """
        Removes the cylinder with the ID provided in the **cylinder_id** argument.

        Args:
            cylinder_id: An integer ID of the cylinder to be deleted.
        """
        data: StrDict = dict(type="RemoveDebugObjects")
        data["objType"] = "cylinders"
        data["objIDs"] = [cylinder_id]
        self._send(data).ack("DebugObjectsRemoved")

    def add_triangle(
        self,
        vertices: List[Float3],
        rgba_color: Color,
        cling: bool = False,
        offset: float = 0.0,
    ) -> int:
        """
        Adds graphical debug triangle to the simulator with points at positions specified by the
        **vertices** argument.

        Args:
            vertices: List of three ``(x, y, z)`` coordinates of the triangle points.
            rgba_color: A single color of the points of the debug triangle, in the format of ``(R, G, B, A)``.
                        An ``A`` of 1.0 means full visibility, 0.0 means full transparency. Can also be instance
                        of any type that the :func:`coerce_color() <beamngpy.misc.colors.coerce_color>` function accepts.
            cling: Whether or not to align the ``z`` coordinate of the triangle points to the ground.
            offset: The z-axis offset of the triangle coordinates. Can only be used together with
                    ``cling=True`` to spawn triangle an exact amount above the ground.

        Returns:
            An integer ID of the debug triangle added. This ID can be passed to the :func:`remove_triangle` function.
        """
        if not len(vertices) == 3:
            raise BNGValueError("`vertices` needs to be a list of length 3!")

        data: StrDict = dict(type="AddDebugTriangle")
        data["vertices"] = vertices
        data["color"] = coerce_color(rgba_color, alpha=1.0)
        data["cling"] = cling
        data["offset"] = offset
        resp = self._send(data).recv("DebugTriangleAdded")
        return int(resp["triangleID"])

    def remove_triangle(self, triangle_id: int) -> None:
        """
        Removes the triangle with the ID provided in the **triangle_id** argument.

        Args:
            triangle_id: An integer ID of the triangle to be deleted.
        """
        data: StrDict = dict(type="RemoveDebugObjects")
        data["objType"] = "triangles"
        data["objIDs"] = [triangle_id]
        self._send(data).ack("DebugObjectsRemoved")

    def add_rectangle(
        self,
        vertices: List[Float3],
        rgba_color: Color,
        cling: bool = False,
        offset: float = 0.0,
    ) -> int:
        """
        Adds graphical debug rectangle to the simulator with points at positions specified by the
        **vertices** argument.

        Args:
            vertices: List of four ``(x, y, z)`` coordinates of the rectangle points.
            rgba_color: A single color of the points of the debug rectangle, in the format of ``(R, G, B, A)``.
                        An ``A`` of 1.0 means full visibility, 0.0 means full transparency. Can also be instance
                        of any type that the :func:`coerce_color() <beamngpy.misc.colors.coerce_color>` function accepts.
            cling: Whether or not to align the ``z`` coordinate of the rectangle points to the ground.
            offset: The z-axis offset of the rectangle coordinates. Can only be used together with
                    ``cling=True`` to spawn rectangle an exact amount above the ground.

        Returns:
            An integer ID of the debug rectangle added. This ID can be passed to the :func:`remove_rectangle` function.
        """
        if not len(vertices) == 4:
            raise BNGValueError("`vertices` needs to be a list of length 4!")

        data: StrDict = dict(type="AddDebugRectangle")
        data["vertices"] = vertices
        data["color"] = coerce_color(rgba_color, alpha=1.0)
        data["cling"] = cling
        data["offset"] = offset
        resp = self._send(data).recv("DebugRectangleAdded")
        return int(resp["rectangleID"])

    def remove_rectangle(self, rectangle_id: int) -> None:
        """
        Removes the rectangle with the ID provided in the **rectangle_id** argument.

        Args:
            rectangle_id: An integer ID of the rectangle to be deleted.
        """
        data: StrDict = dict(type="RemoveDebugObjects")
        data["objType"] = "rectangles"
        data["objIDs"] = [rectangle_id]
        return self._send(data).ack("DebugObjectsRemoved")

    def add_text(
        self,
        origin: Float3,
        content: str,
        rgba_color: Color,
        cling: bool = False,
        offset: float = 0.0,
    ) -> int:
        """
        Adds graphical debug text to the simulator at the position specified by the **origin** argument.

        Args:
            origin: The position of the text as an ``(x, y, z)`` coordinate.
            content: The text that is going to be displayed.
            rgba_color: A single color of the text, in the format of ``(R, G, B, A)``.
                        An ``A`` of 1.0 means full visibility, 0.0 means full transparency. Can also be instance
                        of any type that the :func:`coerce_color() <beamngpy.misc.colors.coerce_color>` function accepts.
            cling: Whether or not to align the ``z`` coordinate of the text to the ground.
            offset: The z-axis offset of the text origin. Can only be used together with
                    ``cling=True`` to spawn the text an exact amount above the ground.

        Returns:
            An integer ID of the text added. This ID can be passed to the :func:`remove_text` function.
        """
        data: StrDict = dict(type="AddDebugText")
        data["origin"] = origin
        data["content"] = content
        data["color"] = coerce_color(rgba_color, alpha=1.0)
        data["cling"] = cling
        data["offset"] = offset
        resp = self._send(data).recv("DebugTextAdded")
        return int(resp["textID"])

    def remove_text(self, text_id: int) -> None:
        """
        Removes the text with the ID provided in the **text_id** argument.

        Args:
            text_id: An integer ID of the text to be deleted.
        """
        data: StrDict = dict(type="RemoveDebugObjects")
        data["objType"] = "text"
        data["objIDs"] = [text_id]
        return self._send(data).ack("DebugObjectsRemoved")

    def add_square_prism(
        self, end_points: List[Float3], end_point_dims: List[Float2], rgba_color: Color
    ) -> int:
        """
        Adds graphical debug square prism to the simulator with the base squares at positions specified by the
        **end_points** argument.

        Args:
            end_points: List of two ``(x, y, z)`` coordinates of the square prism end points.
            end_point_dims: List of two ``(width, height)`` dimensions of the square prism end points.
            rgba_color: A single color of the points of the debug square prism, in the format of ``(R, G, B, A)``.
                        An ``A`` of 1.0 means full visibility, 0.0 means full transparency. Can also be instance
                        of any type that the :func:`coerce_color() <beamngpy.misc.colors.coerce_color>` function accepts.

        Returns:
            An integer ID of the debug square prism added. This ID can be passed to the :func:`remove_square_prism` function.
        """
        if not len(end_points) == 2:
            raise BNGValueError("`end_points` needs to be a list of length 2!")
        if not len(end_point_dims) == 2:
            raise BNGValueError("`end_points` needs to be a list of length 2!")

        data: StrDict = dict(type="AddDebugSquarePrism")
        data["endPoints"] = end_points
        data["dims"] = end_point_dims
        data["color"] = coerce_color(rgba_color, alpha=1.0)
        resp = self._send(data).recv("DebugSquarePrismAdded")
        return int(resp["prismID"])

    def remove_square_prism(self, prism_id: int) -> None:
        """
        Removes the square prism with the ID provided in the **prism_id** argument.

        Args:
            prism_id: An integer ID of the prism to be deleted.
        """
        data: StrDict = dict(type="RemoveDebugObjects")
        data["objType"] = "squarePrisms"
        data["objIDs"] = [prism_id]
        return self._send(data).ack("DebugObjectsRemoved")
