from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from .scenario import ScenarioObject
from .beamngcommon import ack

if TYPE_CHECKING:
    from .beamng import BeamNGpy
    from .types import ConnData, Float3, Quat


class ProceduralMesh(ScenarioObject):
    def __init__(self, pos, name, material, rot_quat=None):
        super(ProceduralMesh, self).__init__(name, name, 'ProceduralMesh',
                                             pos, (1, 1, 1),
                                             rot_quat=rot_quat)
        self.material = material

    def place(self, bng: BeamNGpy):
        raise NotImplementedError()


class ProceduralCylinder(ProceduralMesh):
    def __init__(self, pos: Float3, radius: float, height: float, name: str,
                 rot_quat: Optional[Quat] = None, material: Optional[str] = None):
        """
        Creates a procedurally generated cylinder mesh with the given
        radius and height at the given position and rotation. The material
        can optionally be specified and a name can be assigned for later
        identification.

        Args:
            pos (tuple): (X, Y, Z) coordinate triplet specifying the cylinder's
                         position.
            radius (float): The radius of the cylinder's base circle.
            height (float): The between top and bottom circles of the
                            cylinder.
            name (str): Name for the mesh. Should be unique.
            rot_quat (tuple): Quaternion specifying the cylinder's rotation
            material (str): Optional material name to use as a texture for the
                            mesh.
        """
        super(ProceduralCylinder, self).__init__(pos, name, material,
                                                 rot_quat=rot_quat)
        self.radius = radius
        self.height = height

    @ack('CreatedCylinder')
    def _create(self, bng: BeamNGpy):
        data: ConnData = dict(type='CreateCylinder')
        data['radius'] = self.radius
        data['height'] = self.height
        data['pos'] = self.pos
        data['rot'] = self.rot
        data['name'] = self.name
        data['material'] = self.material
        return bng.send(data)

    def place(self, bng: BeamNGpy):
        self._create(bng)


class ProceduralBump(ProceduralMesh):
    def __init__(self, pos: Float3, width: float, length: float, height: float, upper_length: float,
                 upper_width: float, name: str, rot_quat: Optional[Quat] = None, material: Optional[str] = None):
        """
        Creates a procedurally generated bump with the given properties at the
        given position and rotation. The material can optionally be specified
        and a name can be assigned for later identification.

        Args:
            pos (tuple): (X, Y, Z) coordinate triplet specifying the cylinder's
                         position.
            width (float): The width of the bump, i.e. its size between left
                           and right edges.
            length (float): The length of the bump, i.e. the distances from
                            up and downward slopes.
            height (float): The height of the tip.
            upper_length (float): The length of the tip.
            upper_width (float): The width of the tip.
            name (str): Name for the mesh. Should be unique.
            rot_quat (tuple): Quaternion specifying the bump's rotation
            material (str): Optional material name to use as a texture for the
                            mesh.
        """
        super(ProceduralBump, self).__init__(pos, name, material,
                                             rot_quat=rot_quat)
        self.width = width
        self.length = length
        self.height = height
        self.upper_length = upper_length
        self.upper_width = upper_width

    @ack('CreatedBump')
    def _create(self, bng: BeamNGpy):
        data: ConnData = dict(type='CreateBump')
        data['width'] = self.width
        data['length'] = self.length
        data['height'] = self.height
        data['upperLength'] = self.upper_length
        data['upperWidth'] = self.upper_width
        data['pos'] = self.pos
        data['rot'] = self.rot
        data['name'] = self.name
        data['material'] = self.material
        return bng.send(data)

    def place(self, bng: BeamNGpy):
        self._create(bng)


class ProceduralCone(ProceduralMesh):
    def __init__(self, pos: Float3, radius: float, height: float, name: str,
                 rot_quat: Optional[Quat] = None, material: Optional[str] = None):
        """
        Creates a procedurally generated cone with the given properties at the
        given position and rotation. The material can optionally be specified
        and a name can be assigned for later identification.

        Args:
            pos (tuple): (X, Y, Z) coordinate triplet specifying the cylinder's
                         position.
            radius (float): Radius of the base circle.
            height (float): Distance of the tip to the base circle.
            name (str): Name for the mesh. Should be unique.
            rot_quat (tuple): Quaternion specifying the cone's rotation
            material (str): Optional material name to use as a texture for the
                            mesh.
        """
        super(ProceduralCone, self).__init__(pos, name, material,
                                             rot_quat=rot_quat)
        self.radius = radius
        self.height = height

    @ack('CreatedCone')
    def _create(self, bng: BeamNGpy):
        data: ConnData = dict(type='CreateCone')
        data['radius'] = self.radius
        data['height'] = self.height
        data['material'] = self.material
        data['name'] = self.name
        data['pos'] = self.pos
        data['rot'] = self.rot
        return bng.send(data)

    def place(self, bng: BeamNGpy):
        self._create(bng)


class ProceduralCube(ProceduralMesh):
    def __init__(self, pos: Float3, size: Float3, name: str, rot_quat: Optional[Quat] = None, material: Optional[str] = None):
        """
        Creates a procedurally generated cube with the given properties at the
        given position and rotation. The material can optionally be specified
        and a name can be assigned for later identification.

        Args:
            pos (tuple): (X, Y, Z) coordinate triplet specifying the cylinder's
                         position.
            size (tuple): A triplet specifying the (length, width, height) of
                          the cuboid.
            name (str): Name for the mesh. Should be unique.
            rot_quat (tuple): Quaternion specifying the cube's rotation
            material (str): Optional material name to use as a texture for the
                            mesh.
        """
        super(ProceduralCube, self).__init__(pos, name, material,
                                             rot_quat=rot_quat)
        self.size = size

    @ack('CreatedCube')
    def _create(self, bng: BeamNGpy):
        """
        Creates a procedurally generated cube with the given properties at the
        given position and rotation. The material can optionally be specified
        and a name can be assigned for later identification.

        Args:
            name (str): Name for the mesh. Should be unique.
            size (tuple): A triplet specifying the (length, width, height) of
                          the cuboid.
            pos (tuple): (X, Y, Z) coordinate triplet specifying the cylinder's
                         position.
            rot_quat (tuple): Quaternion specifying the cube's rotation
            material (str): Optional material name to use as a texture for the
                            mesh.
        """
        data: ConnData = dict(type='CreateCube')
        data['size'] = self.size
        data['pos'] = self.pos
        data['rot'] = self.rot
        data['material'] = self.material
        data['name'] = self.name
        return bng.send(data)

    def place(self, bng: BeamNGpy):
        self._create(bng)


class ProceduralRing(ProceduralMesh):
    def __init__(self, pos: Float3, radius: float, thickness: float, name: str,
                 rot_quat:Optional[Quat] = None, material: Optional[str] = None):
        """
        Creates a procedurally generated ring with the given properties at the
        given position and rotation. The material can optionally be specified
        and a name can be assigned for later identification.

        Args:
            pos (tuple): (X, Y, Z) coordinate triplet specifying the cylinder's
                         position.
            radius (float): Radius of the circle encompassing the ring.
            thickness (float): Thickness of the rim.
            name (str): Name for the mesh. Should be unique.
            rot_quat (tuple): Quaternion specifying the ring's rotation
            material (str): Optional material name to use as a texture for the
                            mesh.
        """
        super(ProceduralRing, self).__init__(pos, name, material,
                                             rot_quat=rot_quat)
        self.radius = radius
        self.thickness = thickness

    @ack('CreatedRing')
    def _create(self, bng: BeamNGpy):
        data: ConnData = dict(type='CreateRing')
        data['radius'] = self.radius
        data['thickness'] = self.thickness
        data['pos'] = self.pos
        data['rot'] = self.rot
        data['material'] = self.material
        data['name'] = self.name
        return bng.send(data)

    def place(self, bng: BeamNGpy):
        self._create(bng)