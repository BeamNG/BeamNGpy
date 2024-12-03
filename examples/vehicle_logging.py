from beamngpy import BeamNGpy, Scenario, Vehicle, angle_to_quat
from time import sleep


def main():
    beamng = BeamNGpy("localhost", 25252)
    beamng.open()

    scenario = Scenario("italy", "vehicle logging")

    vehicle = Vehicle("ego_vehicle", model="pickup", license="PYTHON")
    scenario.add_vehicle(
        vehicle,
        pos=(463.45, 1210.24, 167.23),
        rot_quat=angle_to_quat((0, 0, 90)),
    )
    scenario.make(beamng)

    # Set simulator to 60hz temporal resolution
    beamng.settings.set_deterministic(60)
    beamng.scenario.load(scenario)
    beamng.ui.hide_hud()
    beamng.scenario.start()
    # span the vehicle
    vehicle.ai.set_mode("traffic")
    sleep(3.0)
    # vehicle logging
    vehicle.logging.start(
        "data"
    )  # it will save the data in the userfolder/<BeamNG version number>
    sleep(10.0)
    vehicle.logging.stop()
    vehicle.ai.set_mode("disabled")
    beamng.ui.show_hud()
    print("Vehicle data saved to userfolder.")
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
