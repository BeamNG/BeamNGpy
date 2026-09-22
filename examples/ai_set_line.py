"""
Minimal example: built-in AI follows a world-space polyline via ``Vehicle.ai.set_line``.

Each point is ``{"pos": (x, y, z), "speed": v_m_s}``. This does **not** use the
navgraph. Use it when ``drive_using_waypoints`` / ``set_waypoint`` cannot find a
route (e.g. procedural decal roads not on the graph).

Compare:

* ``ai_set_script.py`` — ``set_script`` with ``x, y, z, t`` (time-stamped path).
* ``ai_waypoints.py`` — named navgraph waypoints and ``drive_using_waypoints``.
"""

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging

LINE_SPEED_M_S = 12.0
STEP_M = 15.0
LENGTH_M = 120.0


def build_line():
    line = []
    y = 0.0
    while y <= LENGTH_M:
        line.append({"pos": (0.0, y, 0.0), "speed": LINE_SPEED_M_S})
        y += STEP_M
    return line


def main():
    set_up_simple_logging()

    beamng = BeamNGpy("localhost", 25252)
    beamng.open(launch=True)

    scenario = Scenario("tech_ground", "ai_set_line_demo")
    vehicle = Vehicle("ego", model="etk800", license="AI")
    scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot_quat=(0, 0, 0, 1))
    scenario.make(beamng)

    beamng.settings.set_deterministic(60)
    beamng.control.pause()
    beamng.scenario.load(scenario)
    beamng.scenario.start()

    vehicle.ai.set_line(build_line())

    for _ in range(80):
        beamng.control.step(60)

    beamng.control.resume()
    beamng.disconnect()
    print("Finished ai_set_line demo.")
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
