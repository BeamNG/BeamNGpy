from pathlib import Path
from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging

SCRIPT_DIR = Path(__file__).parent.resolve()
# VSL config CSV (same format as World Editor > Vehicle Signal Logger > Save configuration).
SIGNAL_CONFIG = SCRIPT_DIR / "data" / "vsl_signal_config_example.csv"


def main():
    set_up_simple_logging()

    beamng = BeamNGpy(host="localhost", port=25252)
    bng = beamng.open(launch=True)

    scenario = Scenario(
        "west_coast_usa",
        "vehicle_logging_demo",
        description="Vehicle stats logging demo",
    )

    vehicle = Vehicle("ego_vehicle", model="etk800", license="RED", color="Red")

    scenario.add_vehicle(
        vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795)
    )
    scenario.make(bng)

    bng.settings.set_deterministic(60)  # Set simulator to 60hz temporal resolution

    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    vehicle.connect(bng)

    # Load signals from the editor-style config CSV.
    # Sampling unit matches World Editor dropdown: s (period) or Hz (rate).
    # Pass at most one of sampling_period_s / sampling_rate_hz.
    # Omit both to use settings/samplingPeriodS from the CSV (0.025 s in the example).
    vehicle.logging.start(
        config_path=SIGNAL_CONFIG,
        output_file="log/vehicle_logging_demo.csv",
    )

    # Alternative without a config file (name-only list):
    # vehicle.logging.start(
    #     output_file="log/vehicle_logging_demo_signal_names.csv",
    #     signal_names=(
    #         "vehicleVelocityX",
    #         "vehicleVelocityY",
    #         "vehicleVelocityZ",
    #         "throttle",
    #         "brake",
    #         "steering",
    #     ),
    #     sampling_period_s=0.025,
    # )

    vehicle.ai.set_mode("span")

    for _ in range(6):
        sleep(5)

    vehicle.logging.stop()
    vehicle.ai.set_mode("disabled")
    bng.ui.show_hud()
    input("Press Enter to exit...")
    bng.disconnect()


if __name__ == "__main__":
    main()
