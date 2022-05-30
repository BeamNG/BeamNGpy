"""
.. module:: west_coast_random
    :platform: Windows
    :synopsis: Shows access of LiDAR point cloud data.
.. moduleauthor:: Dave Stark <dstark@beamng.gmbh>
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
    bng = beamng.open(launch=True)

    scenario = Scenario('west_coast_usa', 'lidar_demo',
                        description='Spanning the map with a lidar sensor')

    vehicle = Vehicle('ego_vehicle', model='etk800',
                      licence='RED', color='Red')

    lidar = Lidar(useSharedMemory=True)     # Run using shared memory.
    #lidar = Lidar(useSharedMemory=False)   # Run without using shared memory instead.

    vehicle.attach_sensor('lidar', lidar)

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675),
                         rot=None, rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(bng)

    bng.set_deterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(60)  # With 60hz temporal resolution

    bng.load_scenario(scenario)
    bng.hide_hud()
    bng.start_scenario()

    # Run for 1000 seconds. Every second, we display the LiDAR point cloud data.
    for i in range(1000):

        vehicle.poll_sensors()
        points = lidar.data['points']
        print(points)

        sleep(1)
   
    bng.close()


if __name__ == '__main__':
    main()