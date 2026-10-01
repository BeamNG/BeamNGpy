from __future__ import annotations

from pathlib import Path
from typing import Iterator

import numpy as np
import pytest

from beamngpy import BeamNGpy, Vehicle

from tests.helpers import loaded_scenario, restart_vehicles

SCRIPT_DATA = Path(__file__).resolve().parent.parent / "examples" / "data"


@pytest.fixture(scope="module")
def script_ai_scene(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    vehicle = Vehicle("ego", model="etk800")
    with loaded_scenario(
        beamng, level="smallgrid", name="script_ai_test", label="Script AI test",
        vehicles=[(vehicle, {"pos": (0, 0, 0), "safe_spawn": True})],
        precompile_shaders=False, pause=True,
    ) as (bng, _scenario):
        yield bng, vehicle


@pytest.fixture(autouse=True)
def _reset_shared_scene(script_ai_scene):
    bng, vehicle = script_ai_scene
    restart_vehicles(bng, vehicle)


def _play_and_assert_moved(bng, vehicle, script, *, steps: int, moved_msg: str) -> None:
    vehicle.sensors.poll()
    start_pos = np.array(vehicle.sensors["state"]["pos"])

    vehicle.ai.execute_script(script, start_delay=0.0, no_reset=False)
    for _ in range(steps):
        bng.control.step(30, wait=True)

    vehicle.sensors.poll()
    end_pos = np.array(vehicle.sensors["state"]["pos"])
    assert np.linalg.norm(end_pos - start_pos) > 1.0
    print(moved_msg)


@pytest.mark.parametrize(
    "record_speed,timing_key,recording_name",
    [
        pytest.param(False, "t", "beamngpy_test_script_ai_record_t.json", id="time"),
        pytest.param(True, "v", "beamngpy_test_script_ai_record_v.json", id="velocity"),
    ],
)
def test_script_ai_record_import_replay(script_ai_scene, record_speed, timing_key, recording_name):
    """Record a Script AI path, import the JSON, and replay."""
    bng, vehicle = script_ai_scene
    other_key = "v" if timing_key == "t" else "t"
    recording_path = Path(bng.user_with_version) / recording_name
    if recording_path.exists():
        recording_path.unlink()

    kind = "time-based" if timing_key == "t" else "velocity-based"
    print(f"Recording Script AI path ({kind})...")
    vehicle.ai.start_recording(record_speed=record_speed)
    vehicle.control(throttle=0.4, steering=0.0, brake=0.0)
    for _ in range(90):
        bng.control.step(20, wait=True)
    vehicle.control(throttle=0.0, steering=0.0, brake=0.5)
    bng.control.step(30, wait=True)
    vehicle.ai.stop_recording(recording_name)

    assert recording_path.exists(), f"Recording was not written to {recording_path}"
    print(f"PASS: Recording written to {recording_path}")

    if timing_key == "t":
        print("Importing recorded Script AI file...")
    else:
        print("Importing recorded velocity-based Script AI file...")
    script = vehicle.ai.import_script_ai_file(recording_name)
    assert len(script) >= 3
    for node in script:
        assert {"x", "y", "z", timing_key} <= set(node.keys())
        assert other_key not in node
    if timing_key == "t":
        assert script[-1]["t"] > script[0]["t"]
    else:
        assert any(node["v"] > 0 for node in script)
    print(f"PASS: Imported {len(script)} {timing_key}-based nodes")

    if timing_key == "t":
        print("Replaying recorded script...")
        moved_msg = "PASS: Vehicle moved during replay"
    else:
        print("Replaying recorded velocity-based script...")
        moved_msg = "PASS: Vehicle moved during velocity-based replay"
    _play_and_assert_moved(bng, vehicle, script, steps=120, moved_msg=moved_msg)

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
        _play_and_assert_moved(
            bng, vehicle, script, steps=100,
            moved_msg=f"PASS: Vehicle moved while playing {filename}",
        )
