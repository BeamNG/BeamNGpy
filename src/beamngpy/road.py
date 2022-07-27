from .beamngcommon import BNGValueError
from .scenario import SceneObject


class Road:
    """
    This class represents a DecalRoad in the environment. It contains
    information about the road's material, direction-ness of lanes,
    and geometry of the edges that make up the road.
    """

    def __init__(self, material, rid=None, interpolate=True, default_width=10.0, **options):
        """
        Creates a new road instance using the given material name. The material
        name needs to match a material that is part of the level in the
        simulator this road will be placed in.

        Args:
            material (str): Name of the material this road uses. This affects
                            how the road looks visually and needs to match a
                            material that's part of the level this road is
                            placed in.
            rid (str): Optional string setting this road's name. If specified,
                       needs to be unique with respect to other roads in the
                       level/scenario.
            interpolate (bool): Whether to apply Catmull-Rom spline
                                interpolation to smooth transition between the
                                road's nodes.
            default_width (float): Default width of the road nodes.
        """
        self.default_width = default_width
        self.material = material

        self.rid = rid

        self.drivability = options.get('drivability', 1)
        self.one_way = options.get('one_way', False)
        self.flip_direction = options.get('flip_direction', False)
        self.over_objects = options.get('over_objects', True)
        self.looped = options.get('looped', False)
        self.smoothness = options.get('smoothness', 0.5)
        self.break_angle = options.get('break_angle', 3)
        self.texture_length = options.get('texture_length', 5)
        self.render_priority = options.get('render_priority', 10)

        self.one_way = '1' if self.one_way else '0'
        self.flip_direction = '1' if self.flip_direction else '0'
        self.over_objects = '1' if self.over_objects else '0'
        self.looped = '1' if self.looped else '0'

        if interpolate:
            self.improved_spline = '1'
        else:
            self.improved_spline = '0'
            self.break_angle = 359.9

        self.nodes = list()

    def add_nodes(self, *nodes):
        """
        Adds a list of nodes to this decal road.

        Args:
            nodes (list): List of (x, y, z) or (x, y, z, width) tuples of the
                          road's nodes.
        """
        for node in nodes:
            if len(node) == 3:
                self.nodes.append((*node, self.default_width))
            elif len(node) == 4:
                self.nodes.append(node)
            else:
                raise BNGValueError(
                    'A decal road node should be either a 3-tuple (x, y, z) or a 4-tuple (x, y, z, width).')


class MeshRoad:
    """
    This class represents a MeshRoad in the environment. It contains
    information about the road's materials, direction-ness of lanes,
    and geometry of the edges that make up the road.
    """

    def __init__(self, top_material, bottom_material=None, side_material=None, rid=None,
                 default_width=10.0, default_depth=5.0, **options):
        """
        Creates a new road instance using the given material name. The material
        name needs to match a material that is part of the level in the
        simulator this road will be placed in.

        Args:
            top_material (str): Name of the material this road uses for the top part.
                                This affects how the road looks visually and needs to
                                match a material that's part of the level this road is
                                placed in.
            bottom_material (str): Name of the material this road uses for the bottom part.
                                   Defaults to ``top_material``.
            side_material (str): Name of the material this road uses for the side part.
                                 Defaults to ``top_material``.
            rid (str): Optional string setting this road's name. If specified,
                       needs to be unique with respect to other roads in the
                       level/scenario.
            default_width (float): Default width of the road nodes.
            default_depth (float): Default depth of the road nodes.
        """
        self.default_width = default_width
        self.default_depth = default_depth

        self.rid = rid

        self.top_material = top_material
        self.bottom_material = bottom_material or top_material
        self.side_material = side_material or top_material
        self.texture_length = options.get('texture_length', 5)
        self.break_angle = options.get('break_angle', 3)
        self.width_subdivisions = options.get('width_subdivisions', 0)

        self.nodes = list()

    def add_nodes(self, *nodes):
        """
        Adds a list of nodes to this decal road.

        Args:
            nodes (list): List of (x, y, z), (x, y, z, width) or (x, y, z, width, depth)
                          tuples of the road's nodes.
        """
        for node in nodes:
            if len(node) == 3:
                self.nodes.append(
                    (*node, self.default_width, self.default_depth))
            elif len(node) == 4:
                self.nodes.append((*node, self.default_depth))
            elif len(node) == 5:
                self.nodes.append(node)
            else:
                raise BNGValueError(
                    'A decal road node should be either a 3-tuple (x, y, z), '
                    '4-tuple (x, y, z, width) or a 5-tuple (x, y, z, width, depth).')


class DecalRoad(SceneObject):
    def __init__(self, options):
        super(DecalRoad, self).__init__(options)
        self.lines = options.get('lines', [])

        self.annotation = options.get('annotation', None)
        self.detail = options.get('Detail', None)
        self.material = options.get('Material', None)
        self.break_angle = options.get('breakAngle', None)
        self.drivability = options.get('drivability', None)
        self.flip_direction = options.get('flipDirection', False)
        self.improved_spline = options.get('improvedSpline', False)
        self.lanes_left = options.get('lanesLeft', None)
        self.lanes_right = options.get('lanesRight', None)
        self.one_way = options.get('oneWay', False)
        self.over_objects = options.get('overObjects', False)
