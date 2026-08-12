from __future__ import annotations

from pathlib import Path
from time import sleep
import csv

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
PHYSICS_FREQUENCY = 2000

# Same format as World Editor > Vehicle Signal Logger > Save configuration.
_VSL_CONFIG = (
    Path(__file__).resolve().parent.parent
    / "examples"
    / "data"
    / "vsl_signal_config_example.csv"
)


def test_vehicle_signal_logging(beamng: BeamNGpy):
    output_file = "log/vsl_logging_test.csv"
    steps = 50
    with beamng as bng:
        file_path = Path(beamng.user_with_version) / Path(output_file)
        if file_path.exists():
            file_path.unlink()
        scenario = Scenario("tech_ground", "vsl_logging_test")
        vehicle = Vehicle("ego", model="etk800")
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), safe_spawn=True)
        scenario.make(bng)

        bng.settings.set_deterministic(60)
        # precompile_shaders=False avoids long GPU work during load that can reset the TCP link on some setups.
        bng.scenario.load(scenario, precompile_shaders=False)
        bng.scenario.start()

        vehicle.connect(bng)
        vehicle.logging.start(
            output_file,
            signals=_VSL_SIGNALS,
            steps=steps,
        )
        # tech_ground has no traffic graph — drive inputs + sim steps so signals change while logging.
        vehicle.control(throttle=1.0, steering=0.2)
        for _ in range(5):
            sleep(1)
        vehicle.logging.stop()

        check_csv_file(file_path, steps / PHYSICS_FREQUENCY)


def test_vehicle_signal_logging_from_config(beamng: BeamNGpy):
    output_file="log/vsl_logging_from_config_test.csv"
    steps = 100
    with beamng as bng:
        file_path = Path(beamng.user_with_version) / Path(output_file)
        if file_path.exists():
            file_path.unlink()
        scenario = Scenario("tech_ground", "vsl_logging_from_config_test")
        vehicle = Vehicle("ego", model="etk800")
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), safe_spawn=True)
        scenario.make(bng)

        bng.settings.set_deterministic(60)
        # precompile_shaders=False avoids long GPU work during load that can reset the TCP link on some setups.
        bng.scenario.load(scenario, precompile_shaders=False)
        bng.scenario.start()

        vehicle.connect(bng)
        # signals + default settings come from the config; override path/steps for the test run.
        vehicle.logging.start(
            config_path=_VSL_CONFIG,
            output_file=output_file,
            steps=steps,
        )
        # tech_ground has no traffic graph — drive inputs + sim steps so signals change while logging.
        vehicle.control(throttle=1.0, steering=0.2)
        for _ in range(5):
            sleep(1)
        vehicle.logging.stop()

        check_csv_file(file_path, steps / PHYSICS_FREQUENCY)


def check_csv_file(file_path: Path, row_time_delta: float):
    assert file_path.exists(), f"File {file_path} does not exist"
    assert file_path.stat().st_size > 0, f"File {file_path} is empty"
    allowed_tolerance = 1e-6
    expected_columns = ["timestamp", "throttle"]
    expected_throttle = 1.0
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

        assert abs(throttle - expected_throttle) <= allowed_tolerance, (
            f"Throttle on CSV row {row_number} is {throttle}, expected {expected_throttle}"
        )

        if previous_timestamp is not None:
            actual_delta = timestamp - previous_timestamp
            assert abs(actual_delta - row_time_delta) <= allowed_tolerance, (
                f"Timestamp delta before CSV row {row_number} is {actual_delta}, "
                f"expected {row_time_delta}"
            )
        previous_timestamp = timestamp
