from __future__ import annotations

from typing import List

from beamngpy.logging import BNGValueError
from beamngpy.scenario.scenario_object import SceneObject
from beamngpy.types import Float3, Float4, Float5, StrDict
from beamngpy.utils.prefab import get_uuid


class Road:
    """
    This class represents a DecalRoad in the environment. It contains
    information about the road's material, direction-ness of lanes,
    and geometry of the edges that make up the road.

    Creates a new road instance using the given material name. The material
    name needs to match a material that is part of the level in the
    simulator this road will be placed in.

    Args:
        material: Name of the material this road uses. This affects
                        how the road looks visually and needs to match a
                        material that's part of the level this road is
                        placed in.
        rid: Optional string setting this road's name. If specified,
                    needs to be unique with respect to other roads in the
                    level/scenario.
        interpolate: Whether to apply Catmull-Rom spline
                            interpolation to smooth transition between the
                            road's nodes.
        default_width: Default width of the road nodes.
    """

    @property
    def _uuid(self):
        return get_uuid(f"Road_{self.rid}")

    def __init__(
        self,
        material: str,
        rid: str | None = None,
        interpolate: bool = True,
        default_width: float = 10.0,
        drivability: int = 1,
        one_way: bool = False,
        flip_direction: bool = False,
        over_objects: bool = True,
        looped: bool = False,
        smoothness: float = 0.5,
        break_angle: float = 3,
        texture_length: int = 5,
        render_priority: int = 10,
    ):
        self.default_width = default_width
        self.material = material

        self.rid = rid

        self.drivability = drivability
        self.one_way = one_way
        self.flip_direction = flip_direction
        self.over_objects = over_objects
        self.looped = looped
        self.smoothness = smoothness
        self.break_angle = break_angle
        self.texture_length = texture_length
        self.render_priority = render_priority

        if interpolate:
            self.improved_spline = True
        else:
            self.improved_spline = False
            self.break_angle = 359.9

        self.nodes: List[Float4] = list()

    def add_nodes(self, *nodes: Float3 | Float4) -> None:
        """
        Adds a list of nodes to this decal road.

        Args:
            nodes: List of ``(x, y, z)`` or ``(x, y, z, width)`` tuples of the
                          road's nodes.
        """
        for node in nodes:
            if len(node) == 3:
                self.nodes.append((*node, self.default_width))
            elif len(node) == 4:
                self.nodes.append(node)
            else:
                raise BNGValueError(
                    "A decal road node should be either a 3-tuple (x, y, z) or a 4-tuple (x, y, z, width)."
                )


class MeshRoad:
    """
    This class represents a MeshRoad in the environment. It contains
    information about the road's materials, direction-ness of lanes,
    and geometry of the edges that make up the road.

    Creates a new road instance using the given material name. The material
    name needs to match a material that is part of the level in the
    simulator this road will be placed in.

    Args:
        top_material: Name of the material this road uses for the top part.
                            This affects how the road looks visually and needs to
                            match a material that's part of the level this road is
                            placed in.
        bottom_material: Name of the material this road uses for the bottom part.
                                Defaults to ``top_material``.
        side_material: Name of the material this road uses for the side part.
                                Defaults to ``top_material``.
        rid: Optional string setting this road's name. If specified,
                    needs to be unique with respect to other roads in the
                    level/scenario.
        default_width: Default width of the road nodes.
        default_depth: Default depth of the road nodes.
    """

    @property
    def _uuid(self):
        return get_uuid(f"MeshRoad_{self.rid}")

    def __init__(
        self,
        top_material: str,
        bottom_material: str | None = None,
        side_material: str | None = None,
        rid: str | None = None,
        default_width: float = 10.0,
        default_depth: float = 5.0,
        texture_length: float = 5,
        break_angle: float = 3,
        width_subdivisions: int = 0,
    ):
        self.default_width = default_width
        self.default_depth = default_depth

        self.rid = rid

        self.top_material = top_material
        self.bottom_material = bottom_material or top_material
        self.side_material = side_material or top_material
        self.texture_length = texture_length
        self.break_angle = break_angle
        self.width_subdivisions = width_subdivisions

        self.nodes: List[Float5] = list()

    def add_nodes(self, *nodes: Float3 | Float4 | Float5) -> None:
        """
        Adds a list of nodes to this decal road.

        Args:
            nodes: List of ``(x, y, z)``, ``(x, y, z, width)`` or ``(x, y, z, width, depth)``
                   tuples of the road's nodes.
        """
        for node in nodes:
            if len(node) == 3:
                self.nodes.append((*node, self.default_width, self.default_depth))
            elif len(node) == 4:
                self.nodes.append((*node, self.default_depth))
            elif len(node) == 5:
                self.nodes.append(node)
            else:
                raise BNGValueError(
                    "A decal road node should be either a 3-tuple (x, y, z), "
                    "4-tuple (x, y, z, width) or a 5-tuple (x, y, z, width, depth)."
                )


class DecalRoad(SceneObject):
    def __init__(self, options: StrDict):
        super(DecalRoad, self).__init__(options)
        self.lines = options.get("lines", [])

        self.annotation = options.get("annotation", None)
        self.detail = options.get("Detail", None)
        self.material = options.get("Material", None)
        self.break_angle = options.get("breakAngle", None)
        self.drivability = options.get("drivability", None)
        self.flip_direction = options.get("flipDirection", False)
        self.improved_spline = options.get("improvedSpline", False)
        self.lanes_left = options.get("lanesLeft", None)
        self.lanes_right = options.get("lanesRight", None)
        self.one_way = options.get("oneWay", False)
        self.over_objects = options.get("overObjects", False)
