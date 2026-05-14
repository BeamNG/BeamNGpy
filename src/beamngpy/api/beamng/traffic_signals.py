from __future__ import annotations

import json
from typing import TYPE_CHECKING, Any, Dict, List, Optional

from beamngpy.logging import BNGError

from .base import Api

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy


def _lua_long_json_string(payload: str) -> str:
    """Wrap ``payload`` in a Lua long-bracket literal safe for ``jsonDecode``."""
    for n in range(0, 32):
        eq = "=" * n
        open_b, close_b = "[" + eq + "[", "]" + eq + "]"
        if open_b not in payload and close_b not in payload:
            return open_b + payload + close_b
    raise ValueError("cannot embed JSON in Lua long string")


class TrafficSignalsApi(Api):
    """
    Read and override traffic signals via the game engine ``core_trafficSignals`` module.

    Requires a loaded level with traffic signal data. Uses :meth:`~beamngpy.BeamNGpy.queue_lua_command`
    under the hood; responses are JSON from the simulator's ``jsonEncode``.

    Manual overrides use ``SignalInstance:setStrictState`` (see in-game ``trafficSignals.lua``):
    the automatic sequence timer can overwrite manual states unless you adjust timing or
    disable sequence control as described there.

    Args:
        beamng: An instance of the simulator.
    """

    def _run_json_chunk(self, body: str) -> Any:
        chunk = "return jsonEncode((function()\n" + body + "\nend)())"
        raw = self._beamng.control.queue_lua_command(chunk, response=True)
        if raw is None:
            raise BNGError("traffic signals: empty response from simulator")
        if not isinstance(raw, str):
            raise BNGError(f"traffic signals: unexpected response type {type(raw)!r}")
        try:
            return json.loads(raw)
        except json.JSONDecodeError as exc:
            raise BNGError(f"traffic signals: invalid JSON from simulator: {raw!r}") from exc

    def _unwrap_ok(self, data: Any) -> Any:
        if isinstance(data, dict) and data.get("ok") is False:
            err = data.get("err", "traffic signals GE error")
            raise BNGError(str(err))
        return data

    def get_map_node_signals(self) -> Dict[str, Any]:
        """
        Return the navgraph-oriented signal snapshot (same structure as ``core_trafficSignals.getMapNodeSignals``).

        Keys are road node ids; values nest by outgoing node, then list per-signal entries with
        ``instance``, ``pos``, ``state``, ``action``, etc.
        """
        body = """
  local ts = core_trafficSignals
  if not ts then
    return {ok=false, err="core_trafficSignals not available"}
  end
  return {ok=true, map_node_signals=ts.getMapNodeSignals()}
"""
        data = self._unwrap_ok(self._run_json_chunk(body))
        assert isinstance(data, dict)
        return data["map_node_signals"]

    def list_instances(self) -> List[Dict[str, Any]]:
        """
        List active signal instances with name, current ``state``, optional ``action``, controller/sequence ids, and ``pos``.
        """
        body = """
  local ts = core_trafficSignals
  if not ts then
    return {ok=false, err="core_trafficSignals not available"}
  end
  local out = {}
  for _, inst in ipairs(ts.getSignals()) do
    if not inst._invalid then
      local stateName, stateData = inst:getState()
      local row = {
        name = inst.name,
        state = stateName,
        controller_id = inst.controllerId,
        sequence_id = inst.sequenceId,
        pos = inst.pos,
      }
      if stateData then
        row.action = stateData.action
      end
      table.insert(out, row)
    end
  end
  return {ok=true, instances=out}
"""
        data = self._unwrap_ok(self._run_json_chunk(body))
        assert isinstance(data, dict)
        return data["instances"]

    def set_instance_strict_state(self, instance_name: str, state_index: Optional[int] = None) -> None:
        """
        Force a signal instance to a controller state index, or reset automatic state.

        Args:
            instance_name: ``name`` field of the signal instance (as in the Traffic Signals editor / level data).
            state_index: 1-based controller state index, or ``None`` to clear manual override and refresh from the sequence.
        """
        if state_index is not None:
            if not isinstance(state_index, int) or isinstance(state_index, bool):
                raise ValueError("state_index must be int or None")
            if state_index <= 0:
                raise ValueError("state_index must be positive, or None to reset")
            lua_state = str(state_index)
        else:
            lua_state = "nil"

        blob = _lua_long_json_string(json.dumps({"name": instance_name}, separators=(",", ":")))
        body = f"""
  local ts = core_trafficSignals
  if not ts then
    return {{ok=false, err="core_trafficSignals not available"}}
  end
  local args = jsonDecode({blob})
  local inst = ts.getSignalByName(args.name)
  if not inst then
    return {{ok=false, err="signal not found", name=args.name}}
  end
  inst:setStrictState({lua_state})
  return {{ok=true}}
"""
        self._unwrap_ok(self._run_json_chunk(body))

    def get_timing_snapshot(self) -> Dict[str, Any]:
        """
        Return controller **state durations**, sequence **phase** metadata, and global **timer**
        from ``core_trafficSignals`` (read-only snapshot for debugging / notebooks).

        Structure (keys may vary by level):

        * ``timer``: global signal clock (seconds).
        * ``active``, ``loaded``: engine flags from ``getData()``.
        * ``controllers``: list of ``{name, id, type, totalDuration, states: [{index, state, duration}]}``.
        * ``sequences``: list of ``{name, id, active, currStep, currPhase, totalDuration, ignoreTimer, phases}``.

        Durations are the per-state values used when building the intersection **timeline** in Lua
        (``trafficSignals.lua``). Changing them at runtime is possible via
        :meth:`set_controller_state_duration` but behaviour can be timing-sensitive.
        """
        body = """
  local ts = core_trafficSignals
  if not ts then
    return {ok=false, err="core_trafficSignals not available"}
  end
  local d = ts.getData()
  local controllers = {}
  for _, c in ipairs(ts.getControllers()) do
    local states = {}
    for i, st in ipairs(c.states or {}) do
      table.insert(states, {index = i, state = st.state, duration = st.duration})
    end
    table.insert(controllers, {
      id = c.id,
      name = c.name,
      type = c.type,
      totalDuration = c.totalDuration,
      states = states,
    })
  end
  local sequences = {}
  for _, s in ipairs(ts.getSequences()) do
    local phases = {}
    if s.phases then
      for pi, ph in ipairs(s.phases) do
        table.insert(phases, {
          index = pi,
          startTime = ph.startTime,
          controllerIds = ph.controllerIds,
          totalDuration = ph.totalDuration,
        })
      end
    end
    table.insert(sequences, {
      id = s.id,
      name = s.name,
      active = s.active,
      currStep = s.currStep,
      currPhase = s.currPhase,
      totalDuration = s.totalDuration,
      ignoreTimer = s.ignoreTimer,
      phases = phases,
    })
  end
  return {
    ok = true,
    timer = ts.getTimer(),
    active = d.active,
    loaded = d.loaded,
    controllers = controllers,
    sequences = sequences,
  }
"""
        data = self._unwrap_ok(self._run_json_chunk(body))
        assert isinstance(data, dict)
        return data

    def set_controller_state_duration(
        self, controller_name: str, state_index: int, duration_sec: float
    ) -> None:
        """
        **Experimental:** set ``duration`` on one entry of a signal **controller** state list (1-based index),
        then refresh per-controller totals, sequence ``totalDuration``, and call ``resetTimer()`` in GE.

        This mutates live Lua objects used by ``core_trafficSignals``. Prefer editing signals in the
        World Editor **Traffic Manager / Traffic Signals Editor** for stable timings.

        Args:
            controller_name: ``name`` of the controller (see :meth:`get_timing_snapshot`).
            state_index: 1-based index into ``controller.states`` in Lua.
            duration_sec: New duration in seconds (non-negative; very large values behave like hold).
        """
        if not isinstance(state_index, int) or isinstance(state_index, bool) or state_index < 1:
            raise ValueError("state_index must be a positive int (1-based)")
        if duration_sec < 0:
            raise ValueError("duration_sec must be non-negative")
        blob = _lua_long_json_string(
            json.dumps(
                {"name": controller_name, "idx": state_index, "dur": float(duration_sec)},
                separators=(",", ":"),
            )
        )
        body = f"""
  local ts = core_trafficSignals
  if not ts then
    return {{ok=false, err="core_trafficSignals not available"}}
  end
  local args = jsonDecode({blob})
  local c = ts.getControllerByName(args.name)
  if not c then
    return {{ok=false, err="controller not found", name=args.name}}
  end
  if not c.states or not c.states[args.idx] then
    return {{ok=false, err="state index out of range", idx=args.idx}}
  end
  c.states[args.idx].duration = args.dur
  c:calcDuration()
  for _, seq in ipairs(ts.getSequences()) do
    seq:calcDuration()
  end
  ts.resetTimer()
  return {{ok=true}}
"""
        self._unwrap_ok(self._run_json_chunk(body))
