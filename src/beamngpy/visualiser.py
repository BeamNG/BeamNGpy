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


class LidarVisualiser:

    def __init__(self, points_ceiling):
        self.points_ceiling = points_ceiling
        self.points_count = 0

        self.points = []
        self.dirty = False

        self.pos = [0, 0, 0]

        self.window = None
        self.vertex_buf = 0

    def open(self):
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE)
        glutInitWindowSize(1024, 1024)

        self.window = glutCreateWindow('Lidar')

        glutDisplayFunc(self.render)
        glutIdleFunc(self.idle)
        glutKeyboardFunc(self.on_key)
        glutReshapeFunc(resize)

        init_gl(1024, 1024)

        self.vertex_buf = glGenBuffers(1)

    def on_key(self, name, *args):
        if name == b'a':
            self.pos[2] -= 0.25
        if name == b'd':
            self.pos[2] += 0.25
        if name == b'w':
            self.pos[0] += 0.25
        if name == b's':
            self.pos[0] -= 0.25
        if name == b'r':
            self.pos[1] += 0.25
        if name == b'f':
            self.pos[1] -= 0.25

        print(self.pos)

    def update_points(self, points):
        assert not self.dirty
        self.points = points
        self.points_count = len(points)
        self.dirty = True

        self.idle()

    def step(self):
        glutMainLoopEvent()

    def idle(self):
        if self.dirty:
            verts = np.array(self.points, dtype=np.float32)
            verts = verts / np.max([np.abs(verts.min()), verts.max()])
            print(verts)

            glDeleteBuffers(1, self.vertex_buf)
            self.vertex_buf = glGenBuffers(1)

            glBindBuffer(GL_ARRAY_BUFFER, self.vertex_buf)
            glBufferData(GL_ARRAY_BUFFER, self.points_count *
                         4, verts, GL_DYNAMIC_DRAW)

            glBindBuffer(GL_ARRAY_BUFFER, 0)

            self.dirty = False

        # glutPostRedisplay()

    def render(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_MODELVIEW)

        glLoadIdentity()
        gluLookAt(self.pos[0], self.pos[1], self.pos[2],
                  0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0)

        if not self.dirty and self.points_count > 0:
            glBindBuffer(GL_ARRAY_BUFFER, self.vertex_buf)
            glVertexPointer(3, GL_FLOAT, 0, ctypes.c_void_p(0))
            glEnableClientState(GL_VERTEX_ARRAY)
            glDrawArrays(GL_POINTS, 0, self.points_count)
            glDisableClientState(GL_VERTEX_ARRAY)
            glBindBuffer(GL_ARRAY_BUFFER, 0)

        glutSwapBuffers()
        print('Render finished')
