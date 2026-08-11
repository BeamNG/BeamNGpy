from __future__ import annotations

from pathlib import Path

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


def test_vehicle_signal_logging(beamng: BeamNGpy):
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
        vehicle.logging.start(
            "log/vsl_logging_test.csv",
            signals=_VSL_SIGNALS,
            steps=50,
        )
        # tech_ground has no traffic graph — drive inputs + sim steps so signals change while logging.
        vehicle.control(throttle=1.0, steering=0.2)
        for _ in range(180):
            bng.step(1)
        vehicle.logging.stop()


def test_vehicle_signal_logging_from_config(beamng: BeamNGpy):
    with beamng as bng:
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
            output_file="log/vsl_logging_from_config_test.csv",
            steps=50,
        )
        # tech_ground has no traffic graph — drive inputs + sim steps so signals change while logging.
        vehicle.control(throttle=1.0, steering=0.2)
        for _ in range(180):
            bng.step(1)
        vehicle.logging.stop()
