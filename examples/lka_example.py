from __future__ import annotations

from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.vehicle.lka import LaneKeepingAssist

def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)

    # Create a vehicle.
    vehicle = Vehicle("ego_vehicle", model="etk800", licence="PYTHON", color="White")
    # Create a scenario.
    scenario = Scenario(
        "italy",
        "lka_example",
    )
    # Add the vehicle to the scenario.
    scenario.add_vehicle(vehicle, pos=(1216.629, -824.389, 145.414), rot_quat=(-0.014, 0.012, -0.518, 0.855))
    scenario.make(bng)

    bng.scenario.load(scenario)

    bng.scenario.start()

    lka = LaneKeepingAssist(bng, vehicle, risk_level=2)
    lka.start()

    print("You have 300 seconds to drive around and test out the LKA.")
    for _ in range(300):
        sleep(1)

    lka.stop()

    print("Example finished.")
    bng.disconnect()


# Executing this file will start a scenario where the user can test out the lane keeping assist.
# The user can try to overspeed or start departing from the lane and see how the lane keeping assist reacts.
if __name__ == "__main__":
    main()