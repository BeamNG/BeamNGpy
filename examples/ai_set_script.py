"""
Minimal example: built-in AI follows a time-stamped path via ``Vehicle.ai.set_script``.

Each node is ``{"x": ..., "y": ..., "z": ..., "t": ...}``. Speed comes from the
timestamps. By default the polyline clings to the ground (``z`` is filled in).

Compare:

* ``ai_set_line.py`` — ``set_line`` with ``pos`` + ``speed`` (no timestamps).
* ``ai_waypoints.py`` — named navgraph waypoints and ``drive_using_waypoints``.
"""

import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging

SIZE = 1024


def main():
    set_up_simple_logging()

    beamng = BeamNGpy("localhost", 25252)
    beamng.open(launch=True)

    scenario = Scenario("west_coast_usa", "ai_sine")

    vehicle = Vehicle("ego_vehicle", model="etk800", license="AI")

    orig = (-769.1, 400.8, 142.8)

    scenario.add_vehicle(vehicle, pos=orig, rot_quat=(0, 0, 1, 0))
    scenario.make(beamng)

    beamng.control.pause()
    beamng.settings.set_deterministic(60)

    script = []

    points = []
    point_color = (0, 0, 0, 0.1)
    sphere_coordinates = []
    sphere_radii = []
    sphere_colors = []

    for i in range(2400):
        node = {
            #  Calculate the position as a sine curve that makes the vehicle
            #  drive from left to right. The z-coordinate is not calculated in
            #  any way because `ai.set_script` by default makes the polyline to
            #  follow cling to the ground, meaning the z-coordinate will be
            #  filled in automatically.
            "x": 4 * np.sin(np.radians(i)) + orig[0],
            "y": i * 0.2 + orig[1],
            "z": orig[2],
            #  Calculate timestamps for each node such that the speed between
            #  points has a sinusoidal variance to it.
            "t": (2 * i + (np.abs(np.sin(np.radians(i)))) * 64) / 64,
        }
        script.append(node)
        points.append((node["x"], node["y"], node["z"]))

        if i % 10 == 0:
            sphere_coordinates.append((node["x"], node["y"], node["z"]))
            sphere_radii.append(np.abs(np.sin(np.radians(i))) * 0.25)
            sphere_colors.append((np.sin(np.radians(i)), 0, 0, 0.8))

    beamng.scenario.load(scenario)

    beamng.scenario.start()
    sphere_ids = beamng.debug.add_spheres(
        sphere_coordinates, sphere_radii, sphere_colors, cling=True, offset=0.1
    )
    poly_id = beamng.debug.add_polyline(points, point_color, cling=True, offset=0.1)
    preview_script = script[:12]
    spawn_info = vehicle.ai.get_initial_spawn_position_orientation(preview_script)
    print("get_initial_spawn_position_orientation (first nodes):", spawn_info)
    vehicle.ai.set_script(script)

    for i in range(65):
        beamng.control.step(60)

    beamng.debug.remove_spheres(sphere_ids)
    beamng.debug.remove_polyline(poly_id)
    beamng.control.resume()
    beamng.disconnect()
    print("Scenario finished!")
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
