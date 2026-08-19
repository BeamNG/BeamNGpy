from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple

from beamngpy.types import Quat, StrDict

from .base import Api

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy


def _vec3_xyz(
    v: Any, default: Tuple[float, float, float] = (0.0, 0.0, 0.0)
) -> Tuple[float, float, float]:
    """``vec3`` from techCore: ``{x,y,z}`` (live) or ``[x,y,z]`` (editor / signals.json)."""
    if v is None:
        return default
    if isinstance(v, (list, tuple)):
        if len(v) < 3:
            raise ValueError(f"vec3 sequence too short: {v!r}")
        return float(v[0]), float(v[1]), float(v[2])
    if isinstance(v, dict):
        return float(v["x"]), float(v["y"]), float(v["z"])
    raise TypeError(f"expected vec3 dict or sequence, got {type(v)!r}")


class TrafficSignalsApi(Api):
    """
    Read and override traffic signals via BeamNG.tech ``techCore`` and ``core_trafficSignals``.

    Data matches the Traffic Signals Editor / ``levels/<map>/signals.json``. Requires a loaded
    level with traffic signal data.

    Manual overrides use ``SignalInstance:setStrictState`` (see in-game ``trafficSignals.lua``):
    the automatic sequence timer can overwrite manual states unless you adjust timing or
    disable sequence control as described there.

    Args:
        beamng: An instance of the simulator.
    """

    def get_map_node_signals(self) -> Dict[str, Any]:
        """
        Return the navgraph-oriented signal snapshot (same structure as ``core_trafficSignals.getMapNodeSignals``).

        Keys are road node ids; values nest by outgoing node, then list per-signal entries with
        ``instance``, ``pos``, ``state``, ``action``, etc.
        """
        resp = self._send(dict(type="GetTrafficSignalMapNodes")).recv(
            "TrafficSignalMapNodes"
        )
        return resp["data"]

    def list_instances(self) -> List[Dict[str, Any]]:
        """
        List active signal instances with name, current ``state``, optional ``action``, controller/sequence ids, and ``pos``.
        """
        resp = self._send(dict(type="ListTrafficSignalInstances")).recv(
            "TrafficSignalInstances"
        )
        return resp["data"]

    def get_instance_state(self, instance_name: str) -> Dict[str, Any]:
        """
        Read one signal instance's current logical state (and position / direction).

        Returns:
            Dict with ``name``, ``state``, optional ``action``, ``pos``, ``dir``.
        """
        data: StrDict = dict(
            type="GetTrafficSignalInstanceState", instance_name=instance_name
        )
        resp = self._send(data).recv("TrafficSignalInstanceState")
        return resp["data"]

    @staticmethod
    def _dir_xy_to_rot_quat(dx: float, dy: float) -> Quat:
        """Yaw quaternion (z-up) so vehicle +Y faces ``(dx, dy)``."""
        yaw = math.atan2(dx, dy)
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def get_traffic_light(
        self, instance_name: str, distance_m: float = 12.0
    ) -> Dict[str, Any]:
        """
        Pose for a traffic signal instance (for :meth:`~beamngpy.Vehicle.teleport`).

        Args:
            instance_name: Instance name (e.g. ``trafficLight 1``, ``trafficLight1``).
            distance_m: Metres **before** the signal along its approach direction (0 = at anchor).

        Returns:
            ``name``, ``state``, optional ``action``, ``pos`` / ``rot`` (vehicle pose facing the light),
            ``signal_pos`` / ``signal_dir`` (pole anchor).
        """
        if distance_m < 0:
            raise ValueError("distance_m must be non-negative")
        inst = self.get_instance_state(instance_name)
        sx, sy, sz = _vec3_xyz(inst["pos"])
        dx, dy, dz = _vec3_xyz(inst.get("dir"), (0.0, 1.0, 0.0))
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length < 1e-6:
            dx, dy, dz = 0.0, 1.0, 0.0
        else:
            dx, dy, dz = dx / length, dy / length, dz / length
        pos: Tuple[float, float, float] = (
            sx - dx * distance_m,
            sy - dy * distance_m,
            sz - dz * distance_m,
        )
        rot = self._dir_xy_to_rot_quat(dx, dy)
        return {
            "name": inst["name"],
            "state": inst.get("state"),
            "action": inst.get("action"),
            "pos": pos,
            "rot": rot,
            "signal_pos": (sx, sy, sz),
            "signal_dir": (dx, dy, dz),
        }

    def set_instance_strict_state(
        self, instance_name: str, state_index: Optional[int] = None
    ) -> Dict[str, Any]:
        """
        Force a signal instance to a controller state index, or reset automatic state.

        Args:
            instance_name: ``name`` field of the signal instance (as in the Traffic Signals editor / level data).
            state_index: 1-based controller state index, or ``None`` to clear manual override and refresh from the sequence.

        Returns:
            ``before`` and ``after`` snapshots from :meth:`get_instance_state` (logical state / action).
        """
        if state_index is not None:
            if not isinstance(state_index, int) or isinstance(state_index, bool):
                raise ValueError("state_index must be int or None")
            if state_index <= 0:
                raise ValueError("state_index must be positive, or None to reset")
        data: StrDict = dict(
            type="SetTrafficSignalStrictState",
            instance_name=instance_name,
            state_index=state_index,
        )
        resp = self._send(data).recv("TrafficSignalStrictState")
        return resp["data"]

    def get_editor_signals(self, instance_name: Optional[str] = None) -> Dict[str, Any]:
        """
        Traffic Signals Editor / ``levels/<map>/signals.json`` layout from GE ``onSerialize()``.

        Level data is loaded by ``core_trafficSignals.loadSignals``; the World Editor writes the
        same fields when saving ``signals.json``.

        Args:
            instance_name: If set, return ``instance``, ``controller``, and ``sequence`` for that
                lamp (same rows as in the editor's instance / controller / sequence panels).
                If omitted, return full ``instances``, ``controllers``, and ``sequences`` tables.

        Returns:
            Dict including ``timer`` (global signal clock, seconds). Per-instance results use
            ``controller["states"]`` for the **Duration** column (seconds; ``-1`` = infinite in editor).
            ``sequence["phases"]`` lists intersection phases and ``controllerIds`` like the editor.
        """
        if instance_name is not None and not isinstance(instance_name, str):
            raise TypeError("instance_name must be str or None")
        data: StrDict = dict(type="GetTrafficSignalEditorData")
        if instance_name:
            data["instance_name"] = instance_name
        resp = self._send(data).recv("TrafficSignalEditorData")
        return resp["data"]

    def get_timing_snapshot(self) -> Dict[str, Any]:
        """
        Legacy debug snapshot with runtime sequence step fields.

        Prefer :meth:`get_editor_signals` for controller **Duration** and sequence **phases**
        (same as Traffic Signals Editor / ``signals.json``).
        """
        resp = self._send(dict(type="GetTrafficSignalTimingSnapshot")).recv(
            "TrafficSignalTimingSnapshot"
        )
        return resp["data"]

    def set_controller_state_duration(
        self,
        controller_name: str,
        state_index: int,
        duration_sec: float,
        *,
        instance_name: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Set the **Duration** field on one controller state (Traffic Signals Editor table row).

        Same live object as the editor's controller **States** list: 1-based ``state_index``,
        ``duration_sec`` in seconds (``-1`` = infinite, matching editor save to ``signals.json``).
        Calls ``calcDuration()`` and ``resetTimer()`` like editing timings in GE.

        Does not write ``signals.json``; use the editor **Save** for persistent level data.

        Args:
            controller_name: ``controller["name"]`` from :meth:`get_editor_signals`.
            state_index: 1-based row in ``controller["states"]``.
            duration_sec: New duration (``>= -1``).
            instance_name: If set, included in the returned dict for convenience.

        Returns:
            ``controller`` before/after via ``onSerialize()`` (editor-shaped).
        """
        if not isinstance(state_index, int) or isinstance(state_index, bool) or state_index < 1:
            raise ValueError("state_index must be a positive int (1-based)")
        if duration_sec < -1:
            raise ValueError("duration_sec must be >= -1 (-1 = infinite in editor)")
        data: StrDict = dict(
            type="SetTrafficSignalControllerDuration",
            controller_name=controller_name,
            state_index=state_index,
            duration_sec=float(duration_sec),
        )
        resp = self._send(data).recv("TrafficSignalControllerDuration")
        out = resp["data"]
        if instance_name is not None:
            out = dict(out)
            out["instance_name"] = instance_name
        return out
