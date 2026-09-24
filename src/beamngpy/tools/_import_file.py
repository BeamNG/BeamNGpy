from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from os import PathLike


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
