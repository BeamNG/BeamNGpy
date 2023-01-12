import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging

SIZE = 1024


def main():
    set_up_simple_logging()

    beamng = BeamNGpy('localhost', 64256)
    beamng.open(launch=True)

    scenario = Scenario('west_coast_usa', 'ai_sine')

    vehicle = Vehicle('ego_vehicle', model='etk800', license='AI')

    orig = (-769.1, 400.8, 142.8)

    scenario.add_vehicle(vehicle, pos=orig, rot_quat=(0, 0, 1, 0))
    scenario.make(beamng)

    beamng.settings.set_deterministic(60)

    script = []

    points = []
    point_color = [0, 0, 0, 0.1]
    sphere_coordinates = []
    sphere_radii = []
    sphere_colors = []

    for i in range(3600):
        node = {
            #  Calculate the position as a sine curve that makes the vehicle
            #  drive from left to right. The z-coordinate is not calculated in
            #  any way because `ai_set_script` by default makes the polyline to
            #  follow cling to the ground, meaning the z-coordinate will be
            #  filled in automatically.
            'x': 4 * np.sin(np.radians(i)) + orig[0],
            'y': i * 0.2 + orig[1],
            'z': orig[2],
            #  Calculate timestamps for each node such that the speed between
            #  points has a sinusoidal variance to it.
            't': (2 * i + (np.abs(np.sin(np.radians(i)))) * 64) / 64,
        }
        script.append(node)
        points.append((node['x'], node['y'], node['z']))

        if i % 10 == 0:
            sphere_coordinates.append((node['x'], node['y'], node['z']))
            sphere_radii.append(np.abs(np.sin(np.radians(i))) * 0.25)
            sphere_colors.append((np.sin(np.radians(i)), 0, 0, 0.8))

    try:
        beamng.scenario.load(scenario)

        beamng.scenario.start()
        beamng.debug.add_spheres(sphere_coordinates, sphere_radii, sphere_colors, cling=True, offset=0.1)
        beamng.debug.add_polyline(points, point_color, cling=True, offset=0.1)
        vehicle.ai.set_script(script)

        while True:
            beamng.control.step(60)
    finally:
        beamng.close()


if __name__ == '__main__':
    main()
