"""
.. module:: visualiser
    :platform: Windows
    :synopsis: Implements little sensor visualisation helpers

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
"""

import ctypes
import random
import sys

import numpy as np

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

CLEAR_COLOUR = (0.1, 0.1, 0.1, 1.0)
MAX_DISTANCE = 120 / 3

MOVE_SPEED = 0.25
TURN_SPEED = 1


def resize(width, height):
    if height == 0:
        height = 1

    glViewport(0, 0, width, height)


class LidarVisualiser:

    def __init__(self, points_ceiling):
        self.points_ceiling = points_ceiling
        self.points_count = 0

        self.points = []
        self.colours = np.zeros(points_ceiling * 3, dtype=np.float32)
        self.dirty = False

        self.focus = [0, 0, 0]
        self.pos = [0, 0, 0]
        self.pitch = 90
        self.yaw = 0

        self.mouse_x = -1
        self.mouse_y = -1

        self.vertex_buf = 0
        self.colour_buf = 0

        self.follow = True
        self.frame = 1

    def init_gl(self, width, height):
        glClearColor(*CLEAR_COLOUR)
        glDepthFunc(GL_LESS)
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_CULL_FACE)

        glPointSize(0.5)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(90, width / height, 0.01, 5000)

        glViewport(0, 0, width, height)

    def open(self, width, height):
        glutDisplayFunc(self.render)
        glutKeyboardFunc(self.on_key)
        glutMotionFunc(self.on_drag)
        glutReshapeFunc(resize)

        self.init_gl(width, height)

        self.vertex_buf = glGenBuffers(1)
        self.colour_buf = glGenBuffers(1)

    def on_key(self, name, *args):
        if name == b'f':
            self.follow = not self.follow

        if self.follow:
            return

        if name == b'w':
            self.pos[0] += self.focus[0]
            self.pos[1] += self.focus[1]
            self.pos[2] += self.focus[2]
        if name == b's':
            self.pos[0] -= self.focus[0]
            self.pos[1] -= self.focus[1]
            self.pos[2] -= self.focus[2]

    def on_drag(self, x, y):
        if self.follow:
            return

        delta_x = self.mouse_x
        delta_y = self.mouse_y
        if delta_x == -1:
            delta_x = x
        if delta_y == -1:
            delta_y = y
        delta_x = x - delta_x
        delta_y = y - delta_y

        self.yaw -= delta_x * TURN_SPEED
        self.pitch += delta_y * TURN_SPEED

        self.pitch = np.clip(self.pitch, 1, 179)

        rad_yaw = np.radians(self.yaw)
        rad_pitch = np.radians(self.pitch)

        self.focus = [
            np.cos(rad_yaw) * np.sin(rad_pitch),
            np.sin(rad_yaw) * np.sin(rad_pitch),
            np.cos(rad_pitch),
        ]

        self.mouse_x = x
        self.mouse_y = y

    def update_points(self, points, vehicle_state):
        assert not self.dirty
        if len(points) == 0:
            return

        self.points = points
        self.points_count = len(points)

        verts = np.array(self.points, dtype=np.float32)

        glDeleteBuffers(1, self.vertex_buf)
        self.vertex_buf = glGenBuffers(1)

        glBindBuffer(GL_ARRAY_BUFFER, self.vertex_buf)
        glBufferData(GL_ARRAY_BUFFER, self.points_count *
                     4, verts, GL_STATIC_DRAW)

        glBindBuffer(GL_ARRAY_BUFFER, 0)

        avg = [
            np.average(points[0::3]),
            np.average(points[1::3]),
            np.average(points[2::3]),
        ]

        min_height = points[2::3].min()
        max_height = np.absolute(points[2::3].max() - min_height)

        self.colours[0:self.points_count:3] = points[2::3]
        self.colours[0:self.points_count:3] -= min_height
        self.colours[0:self.points_count:3] /= max_height
        self.colours[1:self.points_count:3] = 0.25
        self.colours[2:self.points_count:3] = 1.0 - \
            self.colours[0:self.points_count:3]

        glDeleteBuffers(1, self.colour_buf)
        self.colour_buf = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self.colour_buf)
        glBufferData(GL_ARRAY_BUFFER, self.points_count * 4,
                     self.colours, GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

        if self.follow and vehicle_state:
            self.focus = vehicle_state['pos']

            self.pos[0] = self.focus[0] + vehicle_state['dir'][0] * -30
            self.pos[1] = self.focus[1] + vehicle_state['dir'][1] * -30
            self.pos[2] = self.focus[2] + vehicle_state['dir'][2] + 10

    def render(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_MODELVIEW)

        glLoadIdentity()

        if self.follow:
            gluLookAt(self.pos[0], self.pos[1], self.pos[2],
                      self.focus[0],
                      self.focus[1],
                      self.focus[2],
                      0.0, 0.0, 1.0)
        else:
            gluLookAt(self.pos[0], self.pos[1], self.pos[2],
                      self.pos[0] + self.focus[0],
                      self.pos[1] + self.focus[1],
                      self.pos[2] + self.focus[2],
                      0.0, 0.0, 1.0)

        if self.points_count > 0:
            glBindBuffer(GL_ARRAY_BUFFER, self.vertex_buf)
            glVertexPointer(3, GL_FLOAT, 0, ctypes.c_void_p(0))
            glEnableClientState(GL_VERTEX_ARRAY)
            glBindBuffer(GL_ARRAY_BUFFER, self.colour_buf)
            glColorPointer(3, GL_FLOAT, 0, ctypes.c_void_p(0))
            glEnableClientState(GL_COLOR_ARRAY)
            glDrawArrays(GL_POINTS, 0, self.points_count // 3)
            glDisableClientState(GL_VERTEX_ARRAY)
            glDisableClientState(GL_COLOR_ARRAY)
            glBindBuffer(GL_ARRAY_BUFFER, 0)

        glutSwapBuffers()
