from __future__ import annotations

import json
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

import numpy as np
import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging

SCRIPT_DATA = Path(__file__).resolve().parent.parent / "examples" / "data"


@contextmanager
def script_ai_scenario(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    with beamng as bng:
        scenario = Scenario("smallgrid", "script_ai_test")
        vehicle = Vehicle("ego", model="etk800")
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), safe_spawn=True)
        scenario.make(bng)

        bng.settings.set_deterministic(60)
        bng.control.pause()
        bng.scenario.load(scenario, precompile_shaders=False)
        bng.scenario.start()

        print("Script AI test start.")
        yield bng, vehicle

        print("Script AI test complete.")


@pytest.fixture(scope="module")
def script_ai_scene(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    with script_ai_scenario(beamng) as scene:
        yield scene


def test_script_ai_record_time_import_replay(script_ai_scene):
    """Record a time-based Script AI path, import the JSON, and replay."""
    bng, vehicle = script_ai_scene
    recording_name = "beamngpy_test_script_ai_record_t.json"
    recording_path = Path(bng.user_with_version) / recording_name
    if recording_path.exists():
        recording_path.unlink()

    print("Recording Script AI path (time-based)...")
    vehicle.ai.start_recording(record_speed=False)
    vehicle.control(throttle=0.4, steering=0.0, brake=0.0)
    for _ in range(90):
        bng.control.step(20, wait=True)
    vehicle.control(throttle=0.0, steering=0.0, brake=0.5)
    bng.control.step(30, wait=True)
    vehicle.ai.stop_recording(recording_name)

    assert recording_path.exists(), f"Recording was not written to {recording_path}"
    print(f"PASS: Recording written to {recording_path}")

    print("Importing recorded Script AI file...")
    script = vehicle.ai.import_script_ai_file(recording_name)
    assert len(script) >= 3
    for node in script:
        assert {"x", "y", "z", "t"} <= set(node.keys())
        assert "v" not in node
    assert script[-1]["t"] > script[0]["t"]
    print(f"PASS: Imported {len(script)} t-based nodes")

    print("Replaying recorded script...")
    vehicle.sensors.poll()
    start_pos = np.array(vehicle.sensors["state"]["pos"])

    vehicle.ai.execute_script(script, start_delay=0.0, no_reset=False)
    for _ in range(120):
        bng.control.step(30, wait=True)

    vehicle.sensors.poll()
    end_pos = np.array(vehicle.sensors["state"]["pos"])
    assert np.linalg.norm(end_pos - start_pos) > 1.0
    print("PASS: Vehicle moved during replay")

    if recording_path.exists():
        recording_path.unlink()


def test_script_ai_record_velocity_import_replay(script_ai_scene):
    """Record a velocity-based Script AI path, import the JSON, and replay."""
    bng, vehicle = script_ai_scene
    recording_name = "beamngpy_test_script_ai_record_v.json"
    recording_path = Path(bng.user_with_version) / recording_name
    if recording_path.exists():
        recording_path.unlink()

    print("Recording Script AI path (velocity-based)...")
    vehicle.ai.start_recording(record_speed=True)
    vehicle.control(throttle=0.4, steering=0.0, brake=0.0)
    for _ in range(90):
        bng.control.step(20, wait=True)
    vehicle.control(throttle=0.0, steering=0.0, brake=0.5)
    bng.control.step(30, wait=True)
    vehicle.ai.stop_recording(recording_name)

    assert recording_path.exists(), f"Recording was not written to {recording_path}"
    print(f"PASS: Recording written to {recording_path}")

    print("Importing recorded velocity-based Script AI file...")
    script = vehicle.ai.import_script_ai_file(recording_name)
    assert len(script) >= 3
    for node in script:
        assert {"x", "y", "z", "v"} <= set(node.keys())
        assert "t" not in node
    assert any(node["v"] > 0 for node in script)
    print(f"PASS: Imported {len(script)} v-based nodes")

    print("Replaying recorded velocity-based script...")
    vehicle.sensors.poll()
    start_pos = np.array(vehicle.sensors["state"]["pos"])

    vehicle.ai.execute_script(script, start_delay=0.0, no_reset=False)
    for _ in range(120):
        bng.control.step(30, wait=True)

    vehicle.sensors.poll()
    end_pos = np.array(vehicle.sensors["state"]["pos"])
    assert np.linalg.norm(end_pos - start_pos) > 1.0
    print("PASS: Vehicle moved during velocity-based replay")

    if recording_path.exists():
        recording_path.unlink()


def test_script_ai_import_and_execute(script_ai_scene):
    """Import editor Script AI JSONs and play them with execute_script."""
    bng, vehicle = script_ai_scene
    samples = [
        ("t_based_scriptAI.json", "t"),
        ("v_based_scriptAI.json", "v"),
    ]

    for filename, timing_key in samples:
        sample = SCRIPT_DATA / filename
        assert sample.exists(), f"Missing sample script: {sample}"

        print(f"Importing editor sample {filename}...")
        script = vehicle.ai.import_script_ai_file(sample)
        assert len(script) >= 3
        for node in script:
            assert {"x", "y", "z", timing_key} <= set(node.keys())
            if timing_key == "t":
                assert "v" not in node
            else:
                assert "t" not in node
        assert {"dir", "up"} <= set(script[0].keys())
        print(f"PASS: Imported {len(script)} {timing_key}-based nodes")

        print(f"Executing {filename}...")
        vehicle.sensors.poll()
        start_pos = np.array(vehicle.sensors["state"]["pos"])

        vehicle.ai.execute_script(script, start_delay=0.0, no_reset=False)
        for _ in range(100):
            bng.control.step(30, wait=True)

        vehicle.sensors.poll()
        end_pos = np.array(vehicle.sensors["state"]["pos"])
        assert np.linalg.norm(end_pos - start_pos) > 1.0
        print(f"PASS: Vehicle moved while playing {filename}")


if __name__ == "__main__":
    set_up_simple_logging()

    bng = BeamNGpy("localhost", 25252, quit_on_close=False)
    with script_ai_scenario(bng) as scene:
        for test in (
            test_script_ai_record_time_import_replay,
            test_script_ai_record_velocity_import_replay,
            test_script_ai_import_and_execute,
        ):
            print("Running ", test.__name__)
            test(scene)
