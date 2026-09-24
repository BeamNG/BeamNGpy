from __future__ import annotations

import os
from typing import TYPE_CHECKING

from beamngpy.connection import CommBase
from beamngpy.logging import BNGError
from beamngpy.tools._import_file import sumo_pair_texts
from beamngpy.types import StrDict

if TYPE_CHECKING:
    from os import PathLike

    from beamngpy.beamng import BeamNGpy


class SumoImporter(CommBase):
    """Imports a SUMO node/edge file pair into Road Architect."""

    def __init__(self, bng: BeamNGpy):
        super().__init__(bng, None)

    def import_sumo(
        self,
        file_path_or_prefix: str | PathLike[str],
        options: StrDict | None = None,
    ) -> StrDict:
        """
        Import a SUMO network into the loaded level as playable DecalRoads.

        ``file_path_or_prefix`` may be the common file prefix, the ``.nod.xml``
        path, the ``.edg.xml`` path, or a matching ``.xml`` base path. When both
        paired files exist on the BeamNGpy host they are sent as content and
        staged under the simulator VFS; otherwise paths must already be
        VFS-visible. A level must be loaded; World Editor modules are
        initialized automatically if needed (no manual F11).

        By default ``finalize=True``: after import, Road Architect's Render /
        finalise step runs headlessly so roads are usable in-sim without
        opening World Editor. Pass ``finalize=False`` to keep an editable
        Road Architect session.

        Args:
            file_path_or_prefix: Simulator-readable SUMO prefix or paired file.
            options: Road Architect import options. Supported keys include
                ``finalize`` (default ``True``), ``offsetToTerrain``,
                ``verticalOffset``, ``terraform``, ``domainOfInfluence``,
                ``margin``, ``placeMode``, ``placeX``, and ``placeY``.
        """
        path_str = os.fspath(file_path_or_prefix)
        opts: StrDict = dict(options or {})
        if "finalize" not in opts:
            opts["finalize"] = True
        payload: StrDict = {
            "filePath": path_str,
            "options": opts,
        }
        node_content, edge_content = sumo_pair_texts(path_str)
        if node_content is not None and edge_content is not None:
            payload["nodeContent"] = node_content
            payload["edgeContent"] = edge_content
        response = self.send_recv_ge("ImportSumo", **payload)
        result = response["data"]
        if not result.get("success"):
            raise BNGError(result.get("error", "SUMO import failed."))
        return result
