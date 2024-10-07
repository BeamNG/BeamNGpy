from __future__ import annotations

from typing import TYPE_CHECKING, Any

from beamngpy.types import Float3, Quat, StrDict
from beamngpy.utils.prefab import get_uuid
from beamngpy.utils.validation import validate_object_name

if TYPE_CHECKING:
    from beamngpy import BeamNGpy


class ScenarioObject:
    """
    This class is used to represent objects in the simulator's environment. It
    contains basic information like the object type, position, rotation, and
    scale.

    Creates a scenario object with the given parameters.

    Args:
        oid: name of the asset
        name: asset id
        otype: type of the object according to the BeamNG classification
        pos: x, y, and z coordinates
        scale: defining the scale along the x,y, and z axis.
        rot_quat: Quaternion describing the initial orientation. Defaults to None.
    """

    @property
    def _uuid(self):
        return get_uuid(f"{self.type}_{self.id}")

    @staticmethod
    def from_game_dict(d: StrDict) -> ScenarioObject:
        oid = ""
        name = ""
        otype = ""
        pos = (0, 0, 0)
        rot_quat = (0, 0, 0, 0)
        scale = (0, 0, 0)
        if "id" in d:
            oid = d["id"]
            del d["id"]

        if "name" in d:
            name = d["name"]
            del d["name"]

        if "class" in d:
            otype = d["class"]
            del d["class"]

        if "position" in d:
            pos = d["position"]
            del d["position"]

        if "rotation" in d:
            rot_quat = d["rotation"]
            del d["rotation"]

        if "scale" in d:
            scale = d["scale"]
            del d["scale"]

        return ScenarioObject(oid, name, otype, pos, scale, rot_quat, **d)

    def __init__(
        self,
        oid: str,
        name: str | None,
        otype: str,
        pos: Float3,
        scale: Float3,
        rot_quat: Quat | None = None,
        **options: str,
    ):
        self.id = oid
        self.name = name
        self.type = otype
        self.pos = pos
        self.rot = rot_quat
        self.scale = scale
        self.opts = options
        self.children = []

        if self.name:
            validate_object_name(self.name)

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, type(self)):
            return self.id == other.id

        return False

    def __hash__(self) -> int:
        return hash(self.id)

    def __str__(self) -> str:
        return f"{self.type} [{self.id}:{self.name}] @ ({self.pos[0]:5.2f}, {self.pos[1]:5.2f}, {self.pos[2]:5.2f})"

    def __repr__(self) -> str:
        return str(self)

    def remove(self, bng: BeamNGpy) -> None:
        data: StrDict = dict(type="RemoveObject", name=self.name)
        bng._send(data).ack("RemovedObject")


class SceneObject:
    def __init__(self, options: StrDict):
        self.id = options.get("id", None)
        if "id" in options:
            del options["id"]

        self.name = options.get("name", None)
        if "name" in options:
            del options["name"]

        self.type = options.get("class", None)
        if "type" in options:
            del options["type"]

        self.pos = options.get("position", (0, 0, 0))
        if "position" in options:
            del options["position"]

        self.rot = options.get("rotation", (0, 0, 0, 0))
        if "rotation" in options:
            del options["rotation"]

        self.scale = options.get("scale", (0, 0, 0))
        if "scale" in options:
            del options["scale"]

        self.options = options
        self.children = []

    def __eq__(self, other):
        if isinstance(other, type(self)):
            return self.id == other.id

        return False

    def __hash__(self) -> int:
        return hash(self.id)

    def __str__(self) -> str:
        return f"{self.type} [{self.id}:{self.name}] @ ({self.pos[0]:5.2f}, {self.pos[1]:5.2f}, {self.pos[2]:5.2f})"

    def __repr__(self) -> str:
        return str(self)


class StaticObject(ScenarioObject):
    def __init__(
        self,
        name: str,
        pos: Float3,
        scale: Float3,
        shape: str,
        rot_quat: Quat | None = None,
    ):
        super(StaticObject, self).__init__(
            name, None, "TSStatic", pos, scale, rot_quat=rot_quat, shapeName=shape
        )
