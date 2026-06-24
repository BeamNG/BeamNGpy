from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging


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
    vehicle.logging.start_logging(
        "log/vehicle_logging_demo.csv",
        signal_names=(
            "vehicleVelocityX",
            "vehicleVelocityY",
            "vehicleVelocityZ",
            "throttle",
            "brake",
            "steering",
        ),
        frequency_steps=50,
    )

    vehicle.ai.set_mode("span")

    for _ in range(6):
        sleep(5)

    vehicle.logging.stop_logging()
    vehicle.ai.set_mode("disabled")
    bng.ui.show_hud()
    input("Press Enter to exit...")
    bng.disconnect()


if __name__ == "__main__":
    main()
