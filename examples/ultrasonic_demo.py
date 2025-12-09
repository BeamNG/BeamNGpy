from __future__ import annotations

from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Ultrasonic


def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    bng.open(launch=True)

    # Create a vehicle.
    vehicle = Vehicle("ego_vehicle", model="etk800", license="PYTHON", color="Red")

    # Create a scenario.
    scenario = Scenario(
        "gridmap_v2",
        "ultrasonic_demo",
    )
    # Add the vehicle to the scenario.
    scenario.add_vehicle(vehicle, pos=(531.91, 263.99, 100.3), rot_quat=(0.0053479, 0.0082936, 0.7021236, 0.7119867))
    scenario.make(bng)

    bng.scenario.load(scenario)
    bng.scenario.start()

    # NOTE: Create sensor after scenario has started.

    # Create a front-facing ultrasonic sensor.
    ultrasonic = Ultrasonic(
        name="ultrasonic_front",
        bng=bng,
        vehicle=vehicle,
        requested_update_time=0.1,  # Update every 100ms.
        pos=(0, -2.5, 0.5),  # Front of vehicle.
        near_far_planes=(0.1, 5.0),  # Detect objects between 0.1m and 5m.
        range_direct_max_cutoff=5.0,  # Maximum detection range.
        is_visualised=True,  # Show the sensor in the simulator.
        is_snapping_desired=True,  # Snap the sensor to the vehicle.
    )

    print("Ultrasonic sensor active. Polling distance measurements...")
    print("The sensor detects objects in front of the vehicle.\n")

    # Poll the sensor for 60 seconds.
    for _ in range(12):
        sleep(5)
        readings = ultrasonic.poll()
        distance = readings["distance"]
        print(f"Distance measurement: {distance:.2f} meters")

    # Clean up.
    ultrasonic.remove()

    print("\nExample finished.")
    bng.disconnect()


# Executing this file will demonstrate a basic ultrasonic sensor attached to a vehicle.
# The sensor measures the distance to objects in front of the vehicle as it drives.
# The green visualization cone shows the sensor's detection range in the simulator.
if __name__ == "__main__":
    main()

