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

    path = list()
    for i in range(3600):
        pos = (4 * np.sin(np.radians(i)) + orig[0],
               i * 0.2 + orig[1],
               orig[2])
        node = {
            'pos': pos,
            'speed': (np.cos(np.radians(i)) + 1) * 4 + 5
        }
        path.append(node)

    bng = beamng.open(launch=True)
    try:
        bng.load_scenario(scenario)

        bng.start_scenario()
        vehicle.ai_set_line(path)

        while True:
            bng.step(60)
    finally:
        bng.close()


if __name__ == '__main__':
    main()
