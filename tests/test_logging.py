from __future__ import annotations

from beamngpy import BeamNGpy, Scenario, Vehicle

# Kinematics / wheels / powertrain (aligned with ``examples/data/mySignalsList.csv``) plus basic Driver inputs.
_VSL_SIGNAL_NAMES = (
    "throttle",
    "brake",
    "steering",
    "steering_input",
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
            signal_names=_VSL_SIGNAL_NAMES,
            frequency_steps=50,
        )
        # tech_ground has no traffic graph — drive inputs + sim steps so signals change while logging.
        vehicle.control(throttle=1.0, steering=0.2)
        for _ in range(180):
            bng.step(1)
        vehicle.logging.stop()
