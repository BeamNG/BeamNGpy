"""
Minimal example: load west_coast_usa, wait until the level is ready, then cycle
through weather presets.

``set_weather_preset`` only applies presets already loaded by the simulator.
"""

import time

from beamngpy import BeamNGpy, Scenario, Vehicle

LEVEL = "west_coast_usa"
SCENARIO_NAME = "weather_preset_demo"
PRESETS = (
    "cloudy_evening",
    "sunny_noon",
    "sunny_evening",
    "foggy_morning",
    "foggy_night",
    "sunny",
    "rainy",
)
TRANSITION_S = 5.0
HOLD_S = 8.0




def main() -> None:
    beamng = BeamNGpy("localhost", 25252)
    bng = beamng.open(launch=True)

    try:
        scenario = Scenario(LEVEL, SCENARIO_NAME, description="Weather preset demo")
        vehicle = Vehicle("ego_vehicle", model="etk800", license="RED", color="Red")
        scenario.add_vehicle(
            vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795)
        )
        scenario.make(bng)

        print(f"Loading {LEVEL!r}...")
        bng.scenario.load(scenario, precompile_shaders=False)
        bng.ui.hide_hud()
        bng.scenario.start()
        bng.settings.set_deterministic(60)
        bng.scenario.start()
        time.sleep(10)


        for preset in PRESETS:
            print(f"Applying {preset!r}...")
            bng.env.set_weather_preset(preset, time=TRANSITION_S)
            time.sleep(TRANSITION_S + HOLD_S)

        input("Cycle complete. Press Enter to exit...")
    finally:
        bng.ui.show_hud()
        bng.disconnect()


if __name__ == "__main__":
    main()
