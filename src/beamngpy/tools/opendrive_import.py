from __future__ import annotations

import os
from typing import TYPE_CHECKING

from beamngpy.connection import CommBase
from beamngpy.logging import BNGError
from beamngpy.tools._import_file import apply_import_opts, local_text
from beamngpy.types import StrDict

if TYPE_CHECKING:
    from collections.abc import Sequence
    from os import PathLike

    from beamngpy.beamng import BeamNGpy


class OpenDriveImporter(CommBase):
    """Imports OpenDRIVE files into Road Architect in the running simulator."""

    def __init__(self, bng: BeamNGpy):
        super().__init__(bng, None)

    def import_xodr(
        self,
        file_path: str | PathLike[str],
        *,
        placement_xy: Sequence[float] | str | None = None,
        offset_to_terrain: bool = False,
        custom_offset: float | None = None,
        terraform: bool = False,
        domain_of_influence: float = 30.0,
        margin: float = 1.0,
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

        The import takes exactly seven arguments: ``file_path`` and the six
        keyword arguments below. Any other argument raises ``TypeError``;
        invalid argument values raise ``ValueError``.

        ``placement_xy`` — horizontal placement of the network. ``None``
        (default): the first imported road's start node lands at the current
        player vehicle pose. An ``(x, y)`` tuple: a custom world position.
        ``"as_in_file"``: keep the OpenDRIVE plan-view geometry exactly as
        authored in the file. ``"camera_position"``: the first imported
        road's start node lands at the current free-camera position.

        Offset (Z) — mirrors the editor 'Offset (Z)' import options, both off
        by default. ``offset_to_terrain=True`` (editor 'Offset To Terrain')
        drapes the roads onto the existing terrain. ``custom_offset`` in
        metres (editor 'Use Custom Offset') adds that much height to every
        imported node. The two are mutually exclusive; passing both raises
        ``ValueError``.

        ``terraform`` (editor 'Terraform Terrain To Import') terraforms the
        terrain to fit the imported network. It combines with either Offset
        (Z) option or with neither of them. Its extent is set by
        ``domain_of_influence`` (the domain of influence of the terraforming,
        in meters) and ``margin`` (the terraforming margin, around road, in
        meters). Like the editor, both values are always sent with the import;
        their defaults (30 m and 1.0 m) match the editor slider defaults.

        With every Z option off the import applies no vertical handling at
        all: the network keeps the elevations authored in the file.

        Without explicit placement, the import fails with a clear error when
        no current player vehicle exists.

        After import, Road Architect's Render / finalise step runs headlessly
        (DecalRoads, collision, navgraph) so the roads are usable in-sim
        without opening World Editor.

        Args:
            file_path: Path to an ``.xodr`` file readable by the simulator.
            placement_xy: ``None`` (vehicle pose, default), ``(x, y)`` (custom
                world position), ``"as_in_file"`` (file coordinates) or
                ``"camera_position"`` (free-camera position).
            offset_to_terrain: Drape the roads onto the existing terrain
                (editor 'Offset To Terrain'). Default off.
            custom_offset: Use a custom vertical offset: add this many metres
                of height to every imported node (editor 'Use Custom
                Offset'). Default off.
            terraform: Terraform the terrain to fit the imported network
                (editor 'Terraform Terrain To Import'). Default off.
            domain_of_influence: Set the domain of influence of the
                terraforming, in meters. Default 30, the editor default.
            margin: Set the terraforming margin (around road), in meters.
                Default 1.0, the editor default.

        Returns:
            Structured import information, including ``importedRoadCount``,
            ``skippedRoadCount``, ``finalized``, warnings, and unsupported
            feature details.

        Raises:
            BNGError: If Road Architect could not import the file.
        """
        path_str = os.fspath(file_path)
        opts: StrDict = {}
        apply_import_opts(
            opts,
            placement_xy,
            offset_to_terrain,
            custom_offset,
            terraform,
            domain_of_influence,
            margin,
        )
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