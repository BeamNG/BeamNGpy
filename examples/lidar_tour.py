"""
.. module:: lidar_tour
    :platform: Windows
    :synopsis: Example starting in west_coast_usa with a vehicle that has a
               Lidar attached and drives around the environment using the
               builtin AI. Lidar data is displayed using the OpenGL-based
               Lidar visualiser.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""

import sys

from time import sleep


import numpy as np

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.sensors import Lidar
from beamngpy.visualiser import LidarVisualiser

SIZE = 1024


def lidar_resize(width, height):
    if height == 0:
        height = 1

    glViewport(0, 0, width, height)


def open_window(width, height):
    glutInit()
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE)
    glutInitWindowSize(width, height)
    window = glutCreateWindow(b'Lidar Tour')
    lidar_resize(width, height)
    return window


def main():
    setup_logging()

    beamng = BeamNGpy('localhost', 64256)
    scenario = Scenario('west_coast_usa', 'lidar_tour',
                        description='Tour through the west coast gathering '
                                    'Lidar data')

    vehicle = Vehicle('ego_vehicle', model='etk800', licence='LIDAR')
    lidar = Lidar()
    vehicle.attach_sensor('lidar', lidar)

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=None, rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(beamng)

    bng = beamng.open(launch=True)
    try:
        bng.load_scenario(scenario)

        window = open_window(SIZE, SIZE)
        lidar_vis = LidarVisualiser(Lidar.max_points)
        lidar_vis.open(SIZE, SIZE)

        bng.set_steps_per_second(60)
        bng.set_deterministic()

        bng.hide_hud()
        bng.start_scenario()

        bng.pause()
        vehicle.ai_set_mode('span')

        def update():
            sensors = bng.poll_sensors(vehicle)
            points = sensors['lidar']['points']
            bng.step(3, wait=False)

            lidar_vis.update_points(points, vehicle.state)
            glutPostRedisplay()

        glutReshapeFunc(lidar_resize)
        glutIdleFunc(update)
        glutMainLoop()
    finally:
        bng.close()


if __name__ == '__main__':
    main()
