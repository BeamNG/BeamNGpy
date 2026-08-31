"""
Single Script AI Editor example covering the full BeamNGpy surface:

* ``vehicle.ai.start_recording`` / ``stop_recording`` — record a path (t- or v-based)
* ``vehicle.ai.import_script_ai_file`` — load editor / recorded JSON
* ``vehicle.ai.execute_script`` — replay (``start_delay``, ``no_reset``, ``cling``)

Also plays shipped editor samples ``data/t_based_scriptAI.json`` and
``data/v_based_scriptAI.json`` on ``smallgrid``.
"""

import json
from pathlib import Path

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging

SCRIPT_DIR = Path(__file__).parent.resolve()
T_BASED = SCRIPT_DIR / "data" / "t_based_scriptAI.json"
V_BASED = SCRIPT_DIR / "data" / "v_based_scriptAI.json"


def first_node_pos(path: Path):
    with open(path, "r") as f:
        node = json.load(f)["path"][0]
    return (node["x"], node["y"], node["z"])


def play_script(bng, vehicle, script, label, *, start_delay=0.0, no_reset=False):
    kind = "v" if "v" in script[0] else "t"
    print(
        f"Playing {label} ({kind}-based, {len(script)} nodes, "
        f"start_delay={start_delay}, no_reset={no_reset})",
        flush=True,
    )
    start = script[0]
    vehicle.teleport((start["x"], start["y"], start["z"]))
    vehicle.ai.execute_script(
        script, cling=True, start_delay=start_delay, no_reset=no_reset
    )
    for _ in range(80):
        bng.control.step(60)


def main():
    set_up_simple_logging()

    bng = BeamNGpy("localhost", 25252)
    bng.open(launch=True)

    scenario = Scenario("smallgrid", "script_ai_editor_demo")
    vehicle = Vehicle("ego", model="etk800", license="SCRIPT")
    scenario.add_vehicle(vehicle, pos=first_node_pos(T_BASED), safe_spawn=True)
    scenario.make(bng)

    bng.settings.set_deterministic(60)
    bng.control.pause()
    bng.scenario.load(scenario)
    bng.scenario.start()

    recording_name = "script_ai_record_import_replay.json"
    recording_path = Path(bng.user_with_version) / recording_name
    print(f"Recording Script AI path to user folder: {recording_path}", flush=True)

    # record_speed=False (default) writes ``t``; True writes ``v`` (speed-based)
    vehicle.ai.start_recording(record_speed=False)
    vehicle.control(throttle=0.3, steering=0.0, brake=0.0)
    for _ in range(40):
        bng.control.step(60)
    vehicle.control(throttle=0.0, steering=0.0, brake=0.5)
    for _ in range(10):
        bng.control.step(60)
    vehicle.ai.stop_recording(recording_name)
    print(f"Saved recording: {recording_path}", flush=True)

    recorded = vehicle.ai.import_script_ai_file(recording_name)
    play_script(bng, vehicle, recorded, recording_name, start_delay=1.0)

    t_script = vehicle.ai.import_script_ai_file(T_BASED)
    play_script(bng, vehicle, t_script, T_BASED.name)

    v_script = vehicle.ai.import_script_ai_file(V_BASED)
    play_script(bng, vehicle, v_script, V_BASED.name, no_reset=True)

    bng.control.resume()
    bng.disconnect()
    print("Scenario finished!", flush=True)
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
