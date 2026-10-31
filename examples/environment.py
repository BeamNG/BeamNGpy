"""
``BeamNGpy.env`` — time of day, gravity, and weather presets.

``set_weather_preset`` applies named presets from ``art/weather/defaults.json``
(e.g. ``sunny_noon``, ``rainy``). Unknown names are skipped so the demo can
continue.
"""

import time

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging

LEVEL = "west_coast_usa"
PRESETS = (
    "cloudy_evening",
    "sunny_noon",
    "sunny_evening",
    "foggy_morning",
    "foggy_night",
    "sunny",
    "rainy",
)
TRANSITION_S = 10.0
HOLD_S = 1.5




def main() -> None:
    set_up_simple_logging()

    bng = BeamNGpy("localhost", 25252, quit_on_close=False)
    bng.open(launch=True)

    try:
        scenario = Scenario(LEVEL, "environment_demo", description="Environment API demo")
        vehicle = Vehicle("ego_vehicle", model="etk800", license="ENV", color="Red")
        scenario.add_vehicle(
            vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795)
        )
        scenario.make(bng)

        print(f"Loading {LEVEL!r}...")
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        tod = bng.env.get_tod()
        print(f"get_tod keys: {sorted(tod.keys())}")
        print(f"  time: {tod.get('time')} timeStr: {tod.get('timeStr')}")
        bng.env.set_tod(tod=0.35, play=False)
        print("set_tod done")

        g0 = bng.env.get_gravity()
        print(f"get_gravity: {g0}")
        bng.env.set_gravity(-9.807)
        bng.env.set_gravity()
        print("set_gravity done")
        input("Press Enter to exit...")
    finally:
        bng.ui.show_hud()
        bng.disconnect()


if __name__ == "__main__":
    main()
