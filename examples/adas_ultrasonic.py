from __future__ import annotations

from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.vehicle.adas_ultrasonic import AdasUltrasonicApi


def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    bng.open()

    # Create a vehicle.
    vehicle = Vehicle("ego_vehicle", model="etk800", licence="PYTHON", color="Red")
    # Create a scenario.
    scenario = Scenario(
        "gridmap_v2",
        "adas_ultrasonic_example",
    )
    # Add the vehicle to the scenario.
    scenario.add_vehicle(vehicle, pos=(531.91, 263.99, 100.3), rot_quat=(0.0053479, 0.0082936, 0.7021236, 0.7119867))
    scenario.make(bng)
    bng.scenario.load(scenario)
    bng.scenario.start()

    # Ensure the vehicle doesn't use arcade controls
    vehicle.set_shift_mode("realistic_automatic")

    adas_ultrasonic = AdasUltrasonicApi(bng, vehicle)
    adas_ultrasonic.start(
        parkAssist=True, # Use the parking assistant.
        blindSpot=True, # Use the blind spot detection.
        crawl=True, # The current vehicle has an automatic transmission, which moves without throttle when put in gear.
        is_visualised=True # See the ultrasonic sensors in the simulator.
    )

    print("You have 3 minutes to drive around and test out the ADAS.")
    for _ in range(180):
        sleep(1)

    adas_ultrasonic.stop()

    print("Example finished.")
    bng.disconnect()


# Executing this file will start a scenario inside a garage where you can try out the parking assistant.
# Put the car in gear and let it drive into one of the walls or columns.
# It is provided to give example on how to use the parking assistant feature currently available in beamngpy.
if __name__ == "__main__":
    main()