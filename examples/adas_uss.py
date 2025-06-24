from __future__ import annotations

from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.vehicle.adas_uss import AdasUssApi


def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    bng.open()

    # Create a vehicle.
    vehicle = Vehicle("ego_vehicle", model="etk800", licence="PYTHON", color="Red")
    # Create a scenario.
    scenario = Scenario(
        "garage_v2",
        "adas_test",
    )
    # Add the vehicle to the scenario.
    scenario.add_vehicle(vehicle, pos=(0, 0, 102))
    scenario.make(bng)
    # Set simulator to 60hz temporal resolution
    bng.settings.set_deterministic(60)
    bng.scenario.load(scenario)
    bng.scenario.start()

    adas_uss = AdasUssApi(bng, vehicle)
    adas_uss.start(True, True, True)

    print("You have 60 seconds to drive around and test out the ADAS.")
    for _ in range(60):
        sleep(1)

    adas_uss.stop()

    print("Example finished.")
    bng.disconnect()


# Executing this file will start a scenario inside a garage where you can try out the parking assistant.
# Put the car in gear and let it drive into one of the walls or columns.
# It is provided to give example on how to use the parking assistant feature currently available in beamngpy.
if __name__ == "__main__":
    main()