from __future__ import annotations

from .base import VehicleApi


class CouplersApi(VehicleApi):
    """
    An API class gathering couplers-related functionality. To learn more,
    you can check the [Coupler System documentation](https://documentation.beamng.com/modding/vehicle/sections/nodes/couplers/).

    Args:
        vehicle: An instance of a vehicle object.
    """

    def attach(self, nodetag: str | None = None) -> None:
        """
        Attaches the vehicle couplers.

        Args:
            nodetag: The tag of the coupler node.
                     If ``None``, all couplers are attached.
        """
        data = dict(type="AttachCouplers")
        if nodetag is not None:
            data["tag"] = nodetag
        self._send(data)

    def detach(
        self,
        nodetag: str | None = None,
        force_locked: bool = False,
        force_welded: bool = False,
    ) -> None:
        """
        Detaches the vehicle couplers.

        Args:
            nodetag: The tag of the coupler node.
                     If ``None``, all couplers are detached.
            force_locked: Force the locked couplers to be detached.
            force_welded: Force the welded couplers to be detached.
        """
        data = dict(
            type="DetachCouplers", forceLocked=force_locked, forceWelded=force_welded
        )
        if nodetag is not None:
            data["tag"] = nodetag
        self._send(data)

    def toggle(
        self,
        nodetag: str | None = None,
        force_locked: bool = False,
        force_welded: bool = False,
        force_auto_coupling: bool = False,
    ) -> None:
        """
        Toggles the vehicle coupler state.

        Args:
            nodetag: The tag of the coupler node.
                     If ``None``, all couplers are toggled.
            force_locked: Force the locked couplers to be detached.
            force_welded: Force the welded couplers to be detached.
            force_auto_coupling: Whether to force attaching and detaching for couplers with the auto coupling system.
        """
        data = dict(
            type="ToggleCouplers",
            forceLocked=force_locked,
            forceWelded=force_welded,
            forceAutoCoupling=force_auto_coupling,
        )
        if nodetag is not None:
            data["tag"] = nodetag
        self._send(data)
