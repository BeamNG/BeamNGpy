from __future__ import annotations

from typing import TYPE_CHECKING

from beamngpy.types import Float3, Quat, StrDict

from .scenario import ScenarioObject

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy


class ProceduralMesh(ScenarioObject):
    def __init__(
        self, pos: Float3, name: str, material: str | None, rot_quat: Quat | None = None
    ):
        super(ProceduralMesh, self).__init__(
            name, name, "ProceduralMesh", pos, (1, 1, 1), rot_quat=rot_quat
        )
        self.material = material

    def place(self, bng: BeamNGpy) -> None:
        raise NotImplementedError()


class ProceduralCylinder(ProceduralMesh):
    """
    Creates a procedurally generated cylinder mesh with the given
    radius and height at the given position and rotation. The material
    can optionally be specified and a name can be assigned for later
    identification.

    Args:
        pos: (X, Y, Z) coordinate triplet specifying the cylinder's
                        position.
        radius: The radius of the cylinder's base circle.
        height: The between top and bottom circles of the
                        cylinder.
        name: Name for the mesh. Should be unique.
        rot_quat: Quaternion specifying the cylinder's rotation
        material: Optional material name to use as a texture for the mesh.
    """

    def __init__(
        self,
        pos: Float3,
        radius: float,
        height: float,
        name: str,
        rot_quat: Quat | None = None,
        material: str | None = None,
    ):
        super(ProceduralCylinder, self).__init__(pos, name, material, rot_quat=rot_quat)
        self.radius = radius
        self.height = height

    def place(self, bng: BeamNGpy) -> None:
        data: StrDict = dict(type="CreateCylinder")
        data["radius"] = self.radius
        data["height"] = self.height
        data["pos"] = self.pos
        data["rot"] = self.rot
        data["name"] = self.name
        data["material"] = self.material
        bng._send(data).ack("CreatedCylinder")


class ProceduralBump(ProceduralMesh):
    """
    Creates a procedurally generated bump with the given properties at the
    given position and rotation. The material can optionally be specified
    and a name can be assigned for later identification.

    Args:
        pos: (X, Y, Z) coordinate triplet specifying the cylinder's
                        position.
        width: The width of the bump, i.e. its size between left
                        and right edges.
        length: The length of the bump, i.e. the distances from
                        up and downward slopes.
        height: The height of the tip.
        upper_length: The length of the tip.
        upper_width: The width of the tip.
        name: Name for the mesh. Should be unique.
        rot_quat: Quaternion specifying the bump's rotation
        material: Optional material name to use as a texture for the mesh.
    """

    def __init__(
        self,
        pos: Float3,
        width: float,
        length: float,
        height: float,
        upper_length: float,
        upper_width: float,
        name: str,
        rot_quat: Quat | None = None,
        material: str | None = None,
    ):
        super(ProceduralBump, self).__init__(pos, name, material, rot_quat=rot_quat)
        self.width = width
        self.length = length
        self.height = height
        self.upper_length = upper_length
        self.upper_width = upper_width

    def place(self, bng: BeamNGpy) -> None:
        data: StrDict = dict(type="CreateBump")
        data["width"] = self.width
        data["length"] = self.length
        data["height"] = self.height
        data["upperLength"] = self.upper_length
        data["upperWidth"] = self.upper_width
        data["pos"] = self.pos
        data["rot"] = self.rot
        data["name"] = self.name
        data["material"] = self.material
        bng._send(data).ack("CreatedBump")


class ProceduralCone(ProceduralMesh):
    """
    Creates a procedurally generated cone with the given properties at the
    given position and rotation. The material can optionally be specified
    and a name can be assigned for later identification.

    Args:
        pos: (X, Y, Z) coordinate triplet specifying the cylinder's
                        position.
        radius: Radius of the base circle.
        height: Distance of the tip to the base circle.
        name: Name for the mesh. Should be unique.
        rot_quat: Quaternion specifying the cone's rotation
        material: Optional material name to use as a texture for the
                        mesh.
    """

    def __init__(
        self,
        pos: Float3,
        radius: float,
        height: float,
        name: str,
        rot_quat: Quat | None = None,
        material: str | None = None,
    ):
        super(ProceduralCone, self).__init__(pos, name, material, rot_quat=rot_quat)
        self.radius = radius
        self.height = height

    def place(self, bng: BeamNGpy) -> None:
        data: StrDict = dict(type="CreateCone")
        data["radius"] = self.radius
        data["height"] = self.height
        data["material"] = self.material
        data["name"] = self.name
        data["pos"] = self.pos
        data["rot"] = self.rot
        bng._send(data).ack("CreatedCone")


class ProceduralCube(ProceduralMesh):
    """
    Creates a procedurally generated cube with the given properties at the
    given position and rotation. The material can optionally be specified
    and a name can be assigned for later identification.

    Args:
        pos: (X, Y, Z) coordinate triplet specifying the cylinder's
                        position.
        size: A triplet specifying the (length, width, height) of
                        the cuboid.
        name: Name for the mesh. Should be unique.
        rot_quat: Quaternion specifying the cube's rotation
        material: Optional material name to use as a texture for the mesh.
    """

    def __init__(
        self,
        pos: Float3,
        size: Float3,
        name: str,
        rot_quat: Quat | None = None,
        material: str | None = None,
    ):
        super(ProceduralCube, self).__init__(pos, name, material, rot_quat=rot_quat)
        self.size = size

    def place(self, bng: BeamNGpy) -> None:
        data: StrDict = dict(type="CreateCube")
        data["size"] = self.size
        data["pos"] = self.pos
        data["rot"] = self.rot
        data["material"] = self.material
        data["name"] = self.name
        bng._send(data).ack("CreatedCube")


class ProceduralRing(ProceduralMesh):
    """
    Creates a procedurally generated ring with the given properties at the
    given position and rotation. The material can optionally be specified
    and a name can be assigned for later identification.

    Args:
        pos: (X, Y, Z) coordinate triplet specifying the cylinder's
                        position.
        radius: Radius of the circle encompassing the ring.
        thickness: Thickness of the rim.
        name: Name for the mesh. Should be unique.
        rot_quat: Quaternion specifying the ring's rotation
        material: Optional material name to use as a texture for the
                        mesh.
    """

    def __init__(
        self,
        pos: Float3,
        radius: float,
        thickness: float,
        name: str,
        rot_quat: Quat | None = None,
        material: str | None = None,
    ):
        super(ProceduralRing, self).__init__(pos, name, material, rot_quat=rot_quat)
        self.radius = radius
        self.thickness = thickness

    def place(self, bng: BeamNGpy) -> None:
        data: StrDict = dict(type="CreateRing")
        data["radius"] = self.radius
        data["thickness"] = self.thickness
        data["pos"] = self.pos
        data["rot"] = self.rot
        data["material"] = self.material
        data["name"] = self.name
        return bng._send(data).ack("CreatedRing")
