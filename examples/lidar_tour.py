"""
.. module:: lidar_tour
    :platform: Windows
    :synopsis: Example starting in west_coast_usa with a vehicle that has a
               Lidar attached and drives around the environment using the
               builtin AI. Lidar data is displayed using the OpenGL-based
               Lidar visualiser.
.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
"""

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Lidar
from beamngpy.sensors.lidar import MAX_LIDAR_POINTS
from beamngpy.visualiser import LidarVisualiser
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

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
    set_up_simple_logging()

    beamng = BeamNGpy('localhost', 64256)
    bng = beamng.open(launch=True)
    scenario = Scenario('west_coast_usa', 'lidar_tour',
                        description='Tour through the west coast gathering '
                                    'Lidar data')

    vehicle = Vehicle('ego_vehicle', model='etk800', licence='LIDAR')

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675),
                         rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(bng)

    try:
        bng.load_scenario(scenario)

        window = open_window(SIZE, SIZE)
        lidar_vis = LidarVisualiser(MAX_LIDAR_POINTS)
        lidar_vis.open(SIZE, SIZE)

        bng.set_steps_per_second(60)
        bng.set_deterministic()

        bng.hide_hud()
        bng.start_scenario()

        lidar = Lidar('lidar', bng, vehicle, requested_update_time=0.01, is_using_shared_memory=True)     # Send data via shared memory.
        #lidar = Lidar('lidar', bng, vehicle, requested_update_time=0.01, is_using_shared_memory=False)   # Send data through lua socket instead.

        bng.pause()
        vehicle.ai_set_mode('span')

        def update():
            vehicle.poll_sensors()
            points = lidar.poll()['pointCloud']
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
