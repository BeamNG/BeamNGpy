from __future__ import annotations

import os
from typing import TYPE_CHECKING

from beamngpy.connection import CommBase
from beamngpy.logging import BNGError
from beamngpy.tools._import_file import local_text
from beamngpy.types import StrDict

if TYPE_CHECKING:
    from os import PathLike

    from beamngpy.beamng import BeamNGpy


class OpenStreetMapImporter(CommBase):
    """Imports OpenStreetMap files into Road Architect."""

    def __init__(self, bng: BeamNGpy):
        super().__init__(bng, None)

    def import_osm(
        self,
        file_path: str | PathLike[str],
        options: StrDict | None = None,
    ) -> StrDict:
        """
        Import an OSM file into the loaded level as playable DecalRoads.

        ``file_path`` may be a native absolute path on the machine running the
        simulator, or a virtual/game path. When the path exists on the BeamNGpy
        host it is sent as ``fileContent`` and staged under the simulator VFS.
        Otherwise the path must already be VFS-visible. A level must be loaded;
        World Editor modules are initialized automatically if needed (no manual
        F11). Coordinates use the file bounds' minimum longitude and latitude
        as the local origin.

        By default ``finalize=True``: after import, Road Architect's Render /
        finalise step runs headlessly so roads are usable in-sim without
        opening World Editor. Pass ``finalize=False`` to keep an editable
        Road Architect session.

        Args:
            file_path: Path to an ``.osm`` file readable by the simulator.
            options: Road Architect import options. Supported keys include
                ``finalize`` (default ``True``), ``offsetToTerrain``,
                ``verticalOffset``, ``terraform``, ``domainOfInfluence``,
                ``margin``, ``placeMode``, ``placeX``, and ``placeY``.
        """
        path_str = os.fspath(file_path)
        opts: StrDict = dict(options or {})
        if "finalize" not in opts:
            opts["finalize"] = True
        payload: StrDict = {
            "filePath": path_str,
            "options": opts,
        }
        content = local_text(path_str)
        if content is not None:
            payload["fileContent"] = content
        response = self.send_recv_ge("ImportOpenStreetMap", **payload)
        result = response["data"]
        if not result.get("success"):
            raise BNGError(result.get("error", "OpenStreetMap import failed."))
        return result
