"""
.. module:: west_coast_random
    :platform: Windows
    :synopsis: Example code making a scenario in west_coast_usa and having a
               car drive around randomly.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""
import mmap
import random
import sys

from time import sleep

import numpy as np

from matplotlib import pyplot as plt
from matplotlib.pyplot import imshow

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.sensors import Camera, GForces, Lidar, Electrics, Damage, Timer


def main():
    random.seed(1703)

    setup_logging()

    # Plotting code setting up a 3x2 figure
    fig = plt.figure(1, figsize=(10, 5))
    axarr = fig.subplots(2, 3)

    a_colour = axarr[0, 0]
    b_colour = axarr[1, 0]
    a_depth = axarr[0, 1]
    b_depth = axarr[1, 1]
    a_annot = axarr[0, 2]
    b_annot = axarr[1, 2]

    plt.ion()

    beamng = BeamNGpy('localhost', 64256)

    # Create a scenario in west_coast_usa
    scenario = Scenario('west_coast_usa', 'research_test',
                        description='Random driving for research')

    # Set up first vehicle, with two cameras, gforces sensor, lidar, electrical
    # sensors, and damage sensors
    vehicle = Vehicle('ego_vehicle', model='etk800',
                      licence='RED', color='Red')

    # Set up sensors
    pos = (-0.3, 1, 1.0)
    direction = (0, 1, 0)
    fov = 120
    resolution = (512, 512)
    front_camera = Camera(pos, direction, fov, resolution,
                          colour=True, depth=True, annotation=True)
    pos = (0.0, 3, 1.0)
    direction = (0, -1, 0)
    fov = 90
    resolution = (512, 512)
    back_camera = Camera(pos, direction, fov, resolution,
                         colour=True, depth=True, annotation=True)

    gforces = GForces()
    electrics = Electrics()
    damage = Damage()
    lidar = Lidar(visualized=False)
    timer = Timer()

    # Attach them
    vehicle.attach_sensor('front_cam', front_camera)
    vehicle.attach_sensor('back_cam', back_camera)
    vehicle.attach_sensor('gforces', gforces)
    vehicle.attach_sensor('electrics', electrics)
    vehicle.attach_sensor('damage', damage)
    vehicle.attach_sensor('timer', timer)

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=None, rot_quat=(0, 0, 0.3826834, 0.9238795))

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)

    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)
    try:
        bng.hide_hud()
        bng.set_deterministic()  # Set simulator to be deterministic
        bng.set_steps_per_second(60)  # With 60hz temporal resolution

        # Load and start the scenario
        bng.load_scenario(scenario)
        bng.start_scenario()
        # Put simulator in pause awaiting further inputs
        bng.pause()

        assert vehicle.skt

        # Send random inputs to vehice and advance the simulation 20 steps
        for _ in range(1024):
            throttle = random.uniform(0.0, 1.0)
            steering = random.uniform(-1.0, 1.0)
            brake = random.choice([0, 0, 0, 1])
            vehicle.control(throttle=throttle, steering=steering, brake=brake)

            bng.step(20)

            # Retrieve sensor data and show the camera data.
            sensors = bng.poll_sensors(vehicle)

            print('{} seconds passed.'.format(sensors['timer']['time']))

            a_colour.imshow(sensors['front_cam']['colour'].convert('RGB'))
            a_depth.imshow(sensors['front_cam']['depth'].convert('L'))
            a_annot.imshow(sensors['front_cam']['annotation'].convert('RGB'))

            b_colour.imshow(sensors['back_cam']['colour'].convert('RGB'))
            b_depth.imshow(sensors['back_cam']['depth'].convert('L'))
            b_annot.imshow(sensors['back_cam']['annotation'].convert('RGB'))

            plt.pause(0.00001)
    finally:
        bng.close()


if __name__ == '__main__':
    main()
