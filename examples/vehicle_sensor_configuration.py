import random
from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import VehicleSensorConfig


def main():
    random.seed(1703)
    set_up_simple_logging()

    beamng = BeamNGpy("localhost", 25252)
    bng = beamng.open(launch=True)

    scenario = Scenario(
        "west_coast_usa",
        "Vehicle_Sensor_Config_Demo",
        description="Importing a pre-made ADAS sensor suite.",
    )

    vehicle = Vehicle("ego_vehicle", model="pickup", license="ADASConfig")

    scenario.add_vehicle(
        vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795)
    )
    scenario.make(bng)

    bng.settings.set_deterministic(60)  # Set simulator to 60hz temporal resolution

    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    vehicle.ai.set_mode("traffic")

    # Path to config file is relative to user folder ie: /AppData/Local/BeamNG.drive/0.XX/
    config = VehicleSensorConfig("config1", bng, vehicle, "/tech/tconfig.json")

    print("Driving around, polling all sensors of the configuration periodically...")
    for _ in range(20):
        sleep(1)
        for s in range(len(config.sensors)):
            sensor = config.sensors[s]
            print(sensor.name)
            print(sensor.poll())

    config.remove()
    bng.ui.show_hud()
    vehicle.ai.set_mode("disabled")
    print("Scenario finished.")
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
