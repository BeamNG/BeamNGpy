from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
from typing import Iterator
import csv

import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle

_VSL_SIGNALS = [
    "throttle",
    "brake",
    "steering",
    "vehiclePositionX",
    "vehiclePositionY",
    "vehiclePositionZ",
    "vehicleVelocityX",
    "vehicleVelocityY",
    "vehicleVelocityZ",
    "vehicleRoll",
    "wheelSpeedFL",
    "gearIndex",
    "mainEngine outputTorque1",
]

# Same format as World Editor > Vehicle Signal Logger > Save configuration.
_VSL_CONFIG = (
    Path(__file__).resolve().parent.parent
    / "examples"
    / "data"
    / "vsl_signal_config_example.csv"
)


@contextmanager
def logging_scenario(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    with beamng as bng:
        scenario = Scenario("tech_ground", "vsl_logging_test")
        vehicle = Vehicle("ego", model="etk800")
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), safe_spawn=True)
        scenario.make(bng)

        bng.settings.set_deterministic(60)
        # precompile_shaders=False avoids long GPU work during load that can reset the TCP link on some setups.
        bng.scenario.load(scenario, precompile_shaders=False)
        bng.scenario.start()
        vehicle.connect(bng)
        yield bng, vehicle


@pytest.fixture(scope="module")
def logging_scene(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    with logging_scenario(beamng) as scene:
        yield scene


_LOGGING_PHYSICS_STEPS = 300


def _reset_logging_scene(bng: BeamNGpy, vehicle: Vehicle) -> None:
    bng.scenario.restart()
    vehicle.connect(bng)
    vehicle.control(throttle=0.0, brake=0.0, steering=0.0)
    bng.control.pause()
    bng.control.step(1, wait=True)


def _record_logging(bng: BeamNGpy, vehicle: Vehicle) -> None:
    vehicle.control(throttle=1.0, steering=0.2)
    bng.control.step(_LOGGING_PHYSICS_STEPS, wait=True)


def test_vehicle_signal_logging(logging_scene):
    bng, vehicle = logging_scene
    _reset_logging_scene(bng, vehicle)

    output_file = "log/vsl_logging_test.csv"
    steps = 50
    file_path = Path(bng.user_with_version) / Path(output_file)
    if file_path.exists():
        file_path.unlink()

    try:
        vehicle.logging.start(
            output_file,
            signals=_VSL_SIGNALS,
            steps=steps,
        )
        _record_logging(bng, vehicle)
    finally:
        try:
            vehicle.logging.stop()
        finally:
            bng.control.resume()

    check_csv_file(file_path, steps / bng.settings.get_physics_frequency())


def test_vehicle_signal_logging_from_config(logging_scene):
    bng, vehicle = logging_scene
    _reset_logging_scene(bng, vehicle)

    output_file = "log/vsl_logging_from_config_test.csv"
    steps = 100
    file_path = Path(bng.user_with_version) / Path(output_file)
    if file_path.exists():
        file_path.unlink()

    try:
        # signals + default settings come from the config; override path/steps for the test run.
        vehicle.logging.start(
            config_path=_VSL_CONFIG,
            output_file=output_file,
            steps=steps,
        )
        _record_logging(bng, vehicle)
    finally:
        try:
            vehicle.logging.stop()
        finally:
            bng.control.resume()

    check_csv_file(file_path, steps / bng.settings.get_physics_frequency())


def check_csv_file(file_path: Path, row_time_delta: float):
    assert file_path.exists(), f"File {file_path} does not exist"
    assert file_path.stat().st_size > 0, f"File {file_path} is empty"
    allowed_tolerance = 1e-6
    expected_columns = ["timestamp", "throttle"]
    commanded_throttle = 1.0
    min_commanded_row_ratio = 0.5
    min_expected_rows = 10

    with file_path.open(newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        assert reader.fieldnames is not None, f"File {file_path} has no header"
        for column in expected_columns:
            assert column in reader.fieldnames, (
                f"File {file_path} has no {column} column; found {reader.fieldnames}"
            )
        rows = list(reader)

    assert len(rows) >= min_expected_rows, (
        f"File {file_path} has {len(rows)} data rows, expected at least {min_expected_rows}"
    )

    previous_timestamp: float | None = None
    commanded_rows = 0
    for row_number, row in enumerate(rows, start=2):
        try:
            timestamp = float(row["timestamp"])
        except (TypeError, ValueError) as exc:
            raise AssertionError(
                f"Invalid timestamp value on CSV row {row_number}: "
                f"{row['timestamp']!r}"
            ) from exc

        try:
            throttle = float(row["throttle"])
        except (TypeError, ValueError) as exc:
            raise AssertionError(
                f"Invalid throttle value on CSV row {row_number}: {row['throttle']!r}"
            ) from exc

        assert -allowed_tolerance <= throttle <= commanded_throttle + allowed_tolerance, (
            f"Throttle on CSV row {row_number} is {throttle}, expected a value between 0 "
            f"and {commanded_throttle}"
        )
        if abs(throttle - commanded_throttle) <= allowed_tolerance:
            commanded_rows += 1

        if previous_timestamp is not None:
            actual_delta = timestamp - previous_timestamp
            assert abs(actual_delta - row_time_delta) <= allowed_tolerance, (
                f"Timestamp delta before CSV row {row_number} is {actual_delta}, "
                f"expected {row_time_delta}"
            )
        previous_timestamp = timestamp

    assert commanded_rows >= len(rows) * min_commanded_row_ratio, (
        f"File {file_path} has {commanded_rows} of {len(rows)} rows at the commanded throttle "
        f"of {commanded_throttle}, expected at least "
        f"{min_commanded_row_ratio:.0%} of the rows"
    )
