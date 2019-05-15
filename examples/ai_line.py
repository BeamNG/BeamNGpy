import sys

from time import sleep


import numpy as np

from scipy import interpolate

from beamngpy import BeamNGpy, Scenario, Road, Vehicle, setup_logging

SIZE = 1024


def main():
    setup_logging()

    beamng = BeamNGpy('localhost', 64256)
    scenario = Scenario('west_coast_usa', 'ai_sine')

    vehicle = Vehicle('ego_vehicle', model='etk800', licence='AI')

    orig = (-769.1, 400.8, 142.8)

    scenario.add_vehicle(vehicle, pos=orig, rot=(0, 0, 180))
    scenario.make(beamng)

    script = list()
    for i in range(3600):
        node = {
            #  Calculate the position as a sinus curve that makes the vehicle
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

    bng = beamng.open(launch=True)
    try:
        bng.load_scenario(scenario)

        bng.start_scenario()
        vehicle.ai_set_script(script)

        while True:
            bng.step(60)
    finally:
        bng.close()


if __name__ == '__main__':
    main()
