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


def _log(msg: str) -> None:
    print(msg, flush=True)


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

        _log(f"Loading {LEVEL!r}...")
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        tod = bng.env.get_tod()
        _log(f"get_tod keys: {sorted(tod.keys())}")
        _log(f"  time: {tod.get('time')} timeStr: {tod.get('timeStr')}")
        bng.env.set_tod(tod=0.35, play=False)
        _log("set_tod done")

        g0 = bng.env.get_gravity()
        _log(f"get_gravity: {g0}")
        bng.env.set_gravity(-9.807)
        bng.env.set_gravity()
        _log("set_gravity done")

        for preset in PRESETS:
            try:
                _log(f"Applying weather preset {preset!r} (transition={TRANSITION_S}s)...")
                bng.env.set_weather_preset(preset, time=TRANSITION_S)
                _log(f"set_weather_preset({preset!r}) returned")
                time.sleep(HOLD_S)
                _log(f"Held {preset!r} for {HOLD_S}s")
            except Exception as exc:
                _log(f"set_weather_preset({preset!r}) skipped: {exc}")

        _log("Environment demo finished.")
        input("Press Enter to exit...")
    finally:
        bng.ui.show_hud()
        bng.disconnect()


if __name__ == "__main__":
    main()
