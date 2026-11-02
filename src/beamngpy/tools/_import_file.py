from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence
    from os import PathLike

    from beamngpy.types import StrDict


def local_text(path: str | PathLike[str]) -> str | None:
    """Return UTF-8 file text when ``path`` exists on this host, else None."""
    candidate = Path(path)
    if not candidate.is_file():
        return None
    return candidate.read_text(encoding="utf-8")


def sumo_pair_texts(
    file_path_or_prefix: str | PathLike[str],
) -> tuple[str | None, str | None]:
    """Resolve SUMO .nod.xml / .edg.xml and return local texts when both exist."""
    raw = Path(file_path_or_prefix)
    name = raw.name
    if name.endswith(".nod.xml") or name.endswith(".edg.xml"):
        prefix = raw.with_name(name[: -len(".nod.xml")])
    elif name.endswith(".xml"):
        prefix = raw.with_suffix("")
    else:
        prefix = raw
    node_path = Path(str(prefix) + ".nod.xml")
    edge_path = Path(str(prefix) + ".edg.xml")
    if not node_path.is_file() or not edge_path.is_file():
        return None, None
    return (
        node_path.read_text(encoding="utf-8"),
        edge_path.read_text(encoding="utf-8"),
    )


# Sentinel string values accepted by the horizontal placement argument,
# mirroring the Road Architect / Road Network Importer editor options.
AS_IN_FILE = "as_in_file"
CAMERA_POSITION = "camera_position"


def apply_import_opts(
    opts: StrDict,
    placement_xy: Sequence[float] | str | None,
    offset_to_terrain: bool,
    custom_offset: float | None,
    terraform: bool,
    domain_of_influence: float = 30.0,
    margin: float = 1.0,
) -> None:
    """Merge the import arguments into Road Architect import options.

    ``placement_xy`` controls the horizontal placement of the network:
    ``None`` (default: current player vehicle pose), an ``(x, y)`` tuple
    (custom world position for the first road start node), ``AS_IN_FILE``
    (keep the file's own coordinates, editor 'As in file') or
    ``CAMERA_POSITION`` (current free-camera position, editor 'Camera
    position').

    ``offset_to_terrain`` and ``custom_offset`` are the editor 'Offset (Z)'
    options, mutually exclusive like their editor checkboxes:
    ``offset_to_terrain`` drapes the roads onto the existing terrain (editor
    'Offset To Terrain') and ``custom_offset`` adds that many metres of
    height to every imported node (editor 'Use Custom Offset').

    ``terraform`` (editor 'Terraform Terrain To Import') terraforms the
    terrain to fit the imported network; it combines with either 'Offset
    (Z)' option or with neither of them. ``domain_of_influence`` sets the
    domain of influence of the terraforming, in meters; ``margin`` sets the
    terraforming margin (around road), in meters. Like the editor, both
    values are always sent with the import; their defaults (30 m and 1.0 m)
    match the editor slider defaults.

    With the offset/terraform flags off the import applies no vertical
    handling, matching the editor with every checkbox unticked: the network
    imports exactly as read from the file.

    The terraforming values are always written, like the editor (which sends
    them with every import); explicit placement/offset arguments override
    matching raw keys already present in ``opts`` while off/``None``
    arguments only fill in when ``opts`` carries no matching raw keys.
    """
    if placement_xy is None:
        if "placeMode" not in opts:
            opts["placeMode"] = 3
    elif isinstance(placement_xy, str):
        if placement_xy == AS_IN_FILE:
            opts["placeMode"] = 0
        elif placement_xy == CAMERA_POSITION:
            opts["placeMode"] = 2
        else:
            raise ValueError(
                f"placement_xy must be None, an (x, y) tuple, '{AS_IN_FILE}' or "
                f"'{CAMERA_POSITION}', got {placement_xy!r}"
            )
    else:
        x, y = (float(v) for v in placement_xy)
        opts["placeMode"] = 1
        opts["placeX"] = x
        opts["placeY"] = y

    if offset_to_terrain and custom_offset is not None:
        raise ValueError(
            "offset_to_terrain and custom_offset are mutually exclusive "
            "(editor 'Offset To Terrain' and 'Use Custom Offset'); pass one or neither"
        )
    if offset_to_terrain:
        opts["offsetToTerrain"] = True
    elif custom_offset is not None:
        opts["verticalOffset"] = float(custom_offset)

    if terraform:
        opts["terraform"] = True

    if domain_of_influence <= 0:
        raise ValueError(
            f"domain_of_influence must be a positive number of meters, "
            f"got {domain_of_influence!r}"
        )
    opts["domainOfInfluence"] = float(domain_of_influence)
    if margin < 0:
        raise ValueError(
            f"margin must be zero or a positive number of meters, "
            f"got {margin!r}"
        )
    opts["margin"] = float(margin)