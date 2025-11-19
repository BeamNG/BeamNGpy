"""
Example: Run simulation at 2x speed for 30 seconds (simulates 60 seconds).

This demonstrates deterministic mode to speed up simulations for long-term tests. See:
https://documentation.beamng.com/beamng_tech/deterministic_mode/
"""

import time

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging


def main():
    set_up_simple_logging()
    beamng = BeamNGpy("localhost", 25252, quit_on_close=False)
    bng = beamng.open(launch=True)

    try:
        # Setup scenario
        scenario = Scenario("west_coast_usa", "speedup_test")
        vehicle = Vehicle("ego_vehicle", model="etk800", license="FAST", color="Red")
        scenario.add_vehicle(
            vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795)
        )
        scenario.make(bng)
        bng.scenario.load(scenario)
        bng.scenario.start()

        # Start vehicle driving
        vehicle.ai.set_mode("traffic")
        vehicle.ai.set_speed(20.0, mode="limit")
        time.sleep(2)

        # Set 2x speedup: fpslimit=40, factor=2.0 → iterations=1
        fpslimit, factor = 40, 2.0
        iterations = int(20 / fpslimit * factor)
        bng.settings.set_deterministic(steps_per_second=fpslimit, speed_factor=iterations)

        print(f"Running at {factor}x speed for 30s real-time (60s simulation)...")
        time.sleep(30)

        # Reset to normal speed
        bng.settings.set_nondeterministic()
        print("Reset to normal speed. Press Enter to exit...")
        input()

    finally:
        beamng.close()


if __name__ == "__main__":
    main()
