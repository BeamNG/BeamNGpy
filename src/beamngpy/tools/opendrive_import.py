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


class OpenDriveImporter(CommBase):
    """Imports OpenDRIVE files into Road Architect in the running simulator."""

    def __init__(self, bng: BeamNGpy):
        super().__init__(bng, None)

    def import_xodr(
        self,
        file_path: str | PathLike[str],
        options: StrDict | None = None,
    ) -> StrDict:
        """
        Import an OpenDRIVE file into the loaded level as playable DecalRoads.

        ``file_path`` may be a native absolute path on the machine running the
        simulator, or a virtual/game path. When the path exists on the BeamNGpy
        host it is sent as ``fileContent`` and consumed inline by the simulator
        (sandboxed ``io.open`` / ``readFile`` cannot read outside game/user
        mounts). Otherwise the path must already be VFS-visible to the
        simulator. A level must be loaded; World Editor modules are
        initialized automatically if needed (no manual F11).

        By default ``finalize=True``: after import, Road Architect's Render /
        finalise step runs headlessly (DecalRoads, collision, navgraph) so the
        roads are usable in-sim without opening World Editor. Pass
        ``finalize=False`` to leave roads in an editable Road Architect session.

        Args:
            file_path: Path to an ``.xodr`` file readable by the simulator.
            options: Road Architect import options. Supported keys include
                ``finalize`` (default ``True``), ``offsetToTerrain``,
                ``verticalOffset``, ``terraform``, ``domainOfInfluence``,
                ``margin``, ``placeMode``, ``placeX``, and ``placeY``.

        Returns:
            Structured import information, including ``importedRoadCount``,
            ``skippedRoadCount``, ``finalized``, warnings, and unsupported
            feature details.

        Raises:
            BNGError: If Road Architect could not import the file.
        """
        path_str = os.fspath(file_path)
        opts: StrDict = dict(options or {})
        payload: StrDict = {
            "filePath": path_str,
            "options": opts,
        }
        content = local_text(path_str)
        if content is not None:
            payload["fileContent"] = content
        response = self.send_recv_ge("ImportOpenDrive", **payload)
        result = response["data"]
        if not result.get("success"):
            raise BNGError(result.get("error", "OpenDRIVE import failed."))
        return result
