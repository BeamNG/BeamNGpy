"""
.. module:: west_coast_random
    :platform: Windows
    :synopsis: Example code making a scenario in west_coast_usa and having the
               vehicle span the map while emitting Lidar.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""
import mmap
import random
import sys

from time import sleep

import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.sensors import Lidar


def main():
    random.seed(1703)
    setup_logging()

    beamng = BeamNGpy('localhost', 64256)

    scenario = Scenario('west_coast_usa', 'lidar_demo',
                        description='Spanning the map with a lidar sensor')

    vehicle = Vehicle('ego_vehicle', model='etk800',
                      licence='RED', color='Red')

    lidar = Lidar(offset=(0, 0, 1.6))
    vehicle.attach_sensor('lidar', lidar)

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=None, rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(beamng)

    bng = beamng.open(launch=True)
    try:
        bng.set_deterministic()  # Set simulator to be deterministic
        bng.set_steps_per_second(60)  # With 60hz temporal resolution

        bng.load_scenario(scenario)
        bng.hide_hud()
        bng.start_scenario()

        vehicle.ai_set_mode('span')
        print('Driving around for 60 seconds...')
        sleep(60)
    finally:
        bng.close()


if __name__ == '__main__':
    main()
