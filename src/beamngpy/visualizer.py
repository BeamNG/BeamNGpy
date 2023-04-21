import ctypes
from collections import deque
import math
import numpy as np
from PIL import Image
from freetype import *

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from beamngpy.sensors import Camera, Lidar, Ultrasonic, Radar, AdvancedIMU


CLEAR_COLOUR = (0.1, 0.1, 0.1, 1.0)
MAX_DISTANCE = 120 / 3
MOVE_SPEED = 0.25
TURN_SPEED = 1

base, texid = 0, 0
text  = '''Hello World !'''


def _on_resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)

class Visualiser:
    def __init__(self, bng, vehicle, name, width, height, demo, toggle):

        # BeamNG state.
        self.bng = bng
        self.vehicle = vehicle

        # Demonstration state.
        self.demo = demo
        self.toggle = toggle

        # Initialize OpenGL.
        self.width, self.height = width, height
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
        glutInitWindowSize(width, height)
        self.window = glutCreateWindow(name)
        if height == 0:
            height = 1
        glutDisplayFunc(self._on_display)
        glutKeyboardFunc(self.on_key)
        glutMotionFunc(self.on_drag)
        glutReshapeFunc(_on_resize)
        glClearColor(*CLEAR_COLOUR)
        glDepthFunc(GL_LESS)
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_CULL_FACE)
        glPointSize(0.5)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(90, self.width / self.height, 0.01, 5000)
        glViewport(0, 0, self.width, self.height)
        self.vertex_buf = np.uint64(glGenBuffers(1))
        self.colour_buf = np.uint64(glGenBuffers(1))
        glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE )

        # Load font.
        self.makefont( 'SourceCodePro-Regular.ttf', 16 )  # nicest
        #self.makefont( 'AnonymousPro-Regular.ttf', 16 )
        #self.makefont( 'UbuntuMono-B.ttf', 16 )

        # LiDAR initialization.
        self.points_count = 0
        self.points = []
        self.colours = np.zeros(6000000, dtype=np.float32)
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

        self.bitmap_tex = None

        # Camera initialization.
        self.camera_image = None
        self.camera_image_size = None

        # Ultrasonic initialization.
        car_img = Image.open('car.png')
        self.car_img = np.array(car_img)
        self.car_img_size = [car_img.size[1], car_img.size[0]] # note the size is flipped for PIL images.
        half_pi = math.pi * 0.5
        gap = 10.0
        ML_x_offset = 500.0                                                                                                         # Top side-bar.
        ML_y_offset = 665.0
        ML_w = 300.0
        ML_h = 15.0
        ML_h_gap = ML_h + gap
        self.loc_ML_0 = [ML_x_offset, ML_y_offset, ML_x_offset + ML_w, ML_y_offset + ML_h]
        self.loc_ML_1 = [ML_x_offset, ML_y_offset + ML_h_gap, ML_x_offset + ML_w, ML_y_offset + ML_h_gap + ML_h]
        self.loc_ML_2 = [ML_x_offset, ML_y_offset + (2.0 * ML_h_gap), ML_x_offset + ML_w, ML_y_offset + (2.0 * ML_h_gap) + ML_h]
        self.loc_ML_3 = [ML_x_offset, ML_y_offset + (3.0 * ML_h_gap), ML_x_offset + ML_w, ML_y_offset + (3.0 * ML_h_gap) + ML_h]
        self.loc_ML_4 = [ML_x_offset, ML_y_offset + (4.0 * ML_h_gap), ML_x_offset + ML_w, ML_y_offset + (4.0 * ML_h_gap) + ML_h]
        self.loc_ML_5 = [ML_x_offset, ML_y_offset + (5.0 * ML_h_gap), ML_x_offset + ML_w, ML_y_offset + (5.0 * ML_h_gap) + ML_h]
        self.loc_ML_6 = [ML_x_offset, ML_y_offset + (6.0 * ML_h_gap), ML_x_offset + ML_w, ML_y_offset + (6.0 * ML_h_gap) + ML_h]
        MR_x_offset = 500.0                                                                                                         # Bottom side-bar.
        MR_y_offset = 280.0
        MR_w = 300.0
        MR_h = 15.0
        MR_h_gap = MR_h + gap
        self.loc_MR_0 = [MR_x_offset, MR_y_offset, MR_x_offset + MR_w, MR_y_offset - MR_h]
        self.loc_MR_1 = [MR_x_offset, MR_y_offset - MR_h_gap, MR_x_offset + MR_w, MR_y_offset - MR_h_gap - MR_h]
        self.loc_MR_2 = [MR_x_offset, MR_y_offset - (2.0 * MR_h_gap), MR_x_offset + MR_w, MR_y_offset - (2.0 * MR_h_gap) - MR_h]
        self.loc_MR_3 = [MR_x_offset, MR_y_offset - (3.0 * MR_h_gap), MR_x_offset + MR_w, MR_y_offset - (3.0 * MR_h_gap) - MR_h]
        self.loc_MR_4 = [MR_x_offset, MR_y_offset - (4.0 * MR_h_gap), MR_x_offset + MR_w, MR_y_offset - (4.0 * MR_h_gap) - MR_h]
        self.loc_MR_5 = [MR_x_offset, MR_y_offset - (5.0 * MR_h_gap), MR_x_offset + MR_w, MR_y_offset - (5.0 * MR_h_gap) - MR_h]
        self.loc_MR_6 = [MR_x_offset, MR_y_offset - (6.0 * MR_h_gap), MR_x_offset + MR_w, MR_y_offset - (6.0 * MR_h_gap) - MR_h]
        self.div = 100
        div_f = float(self.div)
        self.wid = 15
        cx, cy = 800, 475
        wid_gap = self.wid + gap
        r0, r1, r2, r3, r4, r5, r6 = 190, 190 + wid_gap , 190 + (2 * wid_gap ), 190 + (3 * wid_gap ), 190 + (4 * wid_gap ), 190 + (5 * wid_gap ), 190 + (6 * wid_gap )
        self.TR_tx0, self.TR_tx1, self.TR_tx2, self.TR_tx3, self.TR_tx4, self.TR_tx5, self.TR_tx6 = [], [], [], [], [], [], []      # Top-right arc.
        self.TR_ty0, self.TR_ty1, self.TR_ty2, self.TR_ty3, self.TR_ty4, self.TR_ty5, self.TR_ty6 = [], [], [], [], [], [], []
        for i in range(self.div):
            ang = (i / div_f) * half_pi
            ca, sa = math.cos(ang), math.sin(ang)
            self.TR_tx0.append((r0 * ca) + cx)
            self.TR_tx1.append((r1 * ca) + cx)
            self.TR_tx2.append((r2 * ca) + cx)
            self.TR_tx3.append((r3 * ca) + cx)
            self.TR_tx4.append((r4 * ca) + cx)
            self.TR_tx5.append((r5 * ca) + cx)
            self.TR_tx6.append((r6 * ca) + cx)
            self.TR_ty0.append((r0 * sa) + cy)
            self.TR_ty1.append((r1 * sa) + cy)
            self.TR_ty2.append((r2 * sa) + cy)
            self.TR_ty3.append((r3 * sa) + cy)
            self.TR_ty4.append((r4 * sa) + cy)
            self.TR_ty5.append((r5 * sa) + cy)
            self.TR_ty6.append((r6 * sa) + cy)
        cx, cy = 800, 455
        self.BR_tx0, self.BR_tx1, self.BR_tx2, self.BR_tx3, self.BR_tx4, self.BR_tx5, self.BR_tx6 = [], [], [], [], [], [], []     # Bottom-right arc.
        self.BR_ty0, self.BR_ty1, self.BR_ty2, self.BR_ty3, self.BR_ty4, self.BR_ty5, self.BR_ty6 = [], [], [], [], [], [], []
        for i in range(self.div):
            ang = -(i / div_f) * half_pi
            ca, sa = math.cos(ang), math.sin(ang)
            self.BR_tx0.append((r0 * ca) + cx)
            self.BR_tx1.append((r1 * ca) + cx)
            self.BR_tx2.append((r2 * ca) + cx)
            self.BR_tx3.append((r3 * ca) + cx)
            self.BR_tx4.append((r4 * ca) + cx)
            self.BR_tx5.append((r5 * ca) + cx)
            self.BR_tx6.append((r6 * ca) + cx)
            self.BR_ty0.append((r0 * sa) + cy)
            self.BR_ty1.append((r1 * sa) + cy)
            self.BR_ty2.append((r2 * sa) + cy)
            self.BR_ty3.append((r3 * sa) + cy)
            self.BR_ty4.append((r4 * sa) + cy)
            self.BR_ty5.append((r5 * sa) + cy)
            self.BR_ty6.append((r6 * sa) + cy)
        cx, cy = 485, 475
        self.TL_tx0, self.TL_tx1, self.TL_tx2, self.TL_tx3, self.TL_tx4, self.TL_tx5, self.TL_tx6 = [], [], [], [], [], [], []      # Top-left arc.
        self.TL_ty0, self.TL_ty1, self.TL_ty2, self.TL_ty3, self.TL_ty4, self.TL_ty5, self.TL_ty6 = [], [], [], [], [], [], []
        for i in range(self.div):
            ang = (i / div_f) * half_pi
            ca, sa = math.cos(ang), math.sin(ang)
            self.TL_tx0.append(cx - (r0 * ca))
            self.TL_tx1.append(cx - (r1 * ca))
            self.TL_tx2.append(cx - (r2 * ca))
            self.TL_tx3.append(cx - (r3 * ca))
            self.TL_tx4.append(cx - (r4 * ca))
            self.TL_tx5.append(cx - (r5 * ca))
            self.TL_tx6.append(cx - (r6 * ca))
            self.TL_ty0.append((r0 * sa) + cy)
            self.TL_ty1.append((r1 * sa) + cy)
            self.TL_ty2.append((r2 * sa) + cy)
            self.TL_ty3.append((r3 * sa) + cy)
            self.TL_ty4.append((r4 * sa) + cy)
            self.TL_ty5.append((r5 * sa) + cy)
            self.TL_ty6.append((r6 * sa) + cy)
        cx, cy = 485, 455
        self.BL_tx0, self.BL_tx1, self.BL_tx2, self.BL_tx3, self.BL_tx4, self.BL_tx5, self.BL_tx6 = [], [], [], [], [], [], []      # Bottom-left arc.
        self.BL_ty0, self.BL_ty1, self.BL_ty2, self.BL_ty3, self.BL_ty4, self.BL_ty5, self.BL_ty6 = [], [], [], [], [], [], []
        for i in range(self.div):
            ang = -(i / div_f) * half_pi
            ca, sa = math.cos(ang), math.sin(ang)
            self.BL_tx0.append(cx - (r0 * ca))
            self.BL_tx1.append(cx - (r1 * ca))
            self.BL_tx2.append(cx - (r2 * ca))
            self.BL_tx3.append(cx - (r3 * ca))
            self.BL_tx4.append(cx - (r4 * ca))
            self.BL_tx5.append(cx - (r5 * ca))
            self.BL_tx6.append(cx - (r6 * ca))
            self.BL_ty0.append((r0 * sa) + cy)
            self.BL_ty1.append((r1 * sa) + cy)
            self.BL_ty2.append((r2 * sa) + cy)
            self.BL_ty3.append((r3 * sa) + cy)
            self.BL_ty4.append((r4 * sa) + cy)
            self.BL_ty5.append((r5 * sa) + cy)
            self.BL_ty6.append((r6 * sa) + cy)
        self.us_bar_FL = 6
        self.us_bar_FR = 6
        self.us_bar_BL = 6
        self.us_bar_BR = 6
        self.us_bar_ML = 6
        self.us_bar_MR = 6

        # RADAR initialization.
        self.radar_toggle = 0
        self.radar_image = None
        self.radar_image_size = None
        self.radar_second_image = None
        self.radar_second_image_size = None
        self.radar_res = [800, 800]
        self.radar_bins = [800, 800]
        self.radar_fov = 70.0
        self.radar_range_min = 0.1
        self.radar_range_max = 100.0
        self.fov_azimuth = (self.radar_res[0] / float(self.radar_res[1])) * self.radar_fov
        self.half_fov_azimuth = self.fov_azimuth * 0.5
        self.fov_rad = np.deg2rad(self.fov_azimuth)
        self.max_az_rad = self.fov_rad * 0.5
        self.min_az_rad = -self.max_az_rad
        self.radar_f1 = self.radar_range_max / (self.radar_bins[0] + 1)
        self.radar_f2 = -((self.max_az_rad - self.min_az_rad) / (self.radar_bins[1] + 1))
        self.radar_extent = (-self.half_fov_azimuth, self.half_fov_azimuth, self.radar_range_min, self.radar_range_max)
        self.r, self.az = np.mgrid[0.0:self.radar_range_max:self.radar_f1, self.max_az_rad:self.min_az_rad:self.radar_f2]
        self.az_plus_half_pi = self.az + (np.pi * 0.5)
        self.grid_x = self.r * np.cos(self.az_plus_half_pi)
        self.grid_y = self.r * np.sin(self.az_plus_half_pi)

        # IMU:
        self.imu_acc1_verts, self.imu_acc2_verts, self.imu_acc3_verts = [], [], []
        self.imu_gyro1_verts, self.imu_gyro2_verts, self.imu_gyro3_verts = [], [], []
        self.imu_acc1_axis_x, self.imu_acc1_axis_y = [190, 650, 1210, 650], [200, 640, 200, 910]  # the axis lines for the accel plots.
        self.imu_acc2_axis_x, self.imu_acc2_axis_y = [190, 350, 1210, 350], [200, 340, 200, 610]
        self.imu_acc3_axis_x, self.imu_acc3_axis_y = [190, 50, 1210, 50], [200, 40, 200, 310]
        self.imu_acc1_grid = []
        grid_dx, grid_dy = 10, 5
        for i in range(1, grid_dx + 1):
            div = float(1000) / float(grid_dx)
            gx = 200 + (i * div)
            self.imu_acc1_grid.append([gx, 650, gx, 900])
        for i in range(1, grid_dy + 1):
            div = float(250) / float(grid_dy)
            gy = 650 + (i * div)
            self.imu_acc1_grid.append([200, gy, 1200, gy])
        self.imu_acc2_grid = []
        for i in range(1, grid_dx + 1):
            div = float(1000) / float(grid_dx)
            gx = 200 + (i * div)
            self.imu_acc2_grid.append([gx, 350, gx, 600])
        for i in range(1, grid_dy + 1):
            div = float(250) / float(grid_dy)
            gy = 350 + (i * div)
            self.imu_acc2_grid.append([200, gy, 1200, gy])
        self.imu_acc3_grid = []
        for i in range(1, grid_dx + 1):
            div = float(1000) / float(grid_dx)
            gx = 200 + (i * div)
            self.imu_acc3_grid.append([gx, 50, gx, 300])
        for i in range(1, grid_dy + 1):
            div = float(250) / float(grid_dy)
            gy = 50 + (i * div)
            self.imu_acc3_grid.append([200, gy, 1200, gy])
        self.acc1, self.acc2, self.acc3, self.gyro1, self.gyro2, self.gyro3 = deque(), deque(), deque(), deque(), deque(), deque()
        self.imu_t_start, self.imu_t_end = 200, 1200 # screen limits for all imu readings (in t).
        self.imu_t_width = self.imu_t_end - self.imu_t_start
        self.imu_acc_min, self.imu_acc_max, self.imu_gyro_min, self.imu_gyro_max = -15.0, 15.0, -50.0, 50.0 # value limits for imu readings.
        self.imu_acc_height, self.imu_gyro_height = self.imu_acc_max - self.imu_acc_min, self.imu_gyro_max - self.imu_gyro_min
        self.imu_acc_height_inv = 1.0 / self.imu_acc_height
        self.imu_acc1_y_min, self.imu_acc1_y_max = 650, 900 # screen limits in y for acc1 reading.
        self.imu_acc2_y_min, self.imu_acc2_y_max = 350, 600 # screen limits in y for acc2 reading.
        self.imu_acc3_y_min, self.imu_acc3_y_max = 50, 300 # screen limits in y for acc3 reading.
        self.imu_acc1_y_range = self.imu_acc1_y_max - self.imu_acc1_y_min
        self.imu_acc2_y_range = self.imu_acc2_y_max - self.imu_acc2_y_min
        self.imu_acc3_y_range = self.imu_acc3_y_max - self.imu_acc3_y_min
        self.imu_acc1_f, self.imu_acc2_f, self.imu_acc3_f = self.imu_acc_height_inv * self.imu_acc1_y_range, self.imu_acc_height_inv * self.imu_acc2_y_range, self.imu_acc_height_inv * self.imu_acc3_y_range
        self.imu_t = [] # pre-compute the time axis.
        num_imu_entries = 2000
        for i in range(num_imu_entries):   # populate a queue with 2000 dummy entries (rep. 2 seconds of data) to get us started.
            self.acc1.append(0.0)
            self.acc2.append(0.0)
            self.acc3.append(0.0)
            self.gyro1.append(0.0)
            self.gyro2.append(0.0)
            self.gyro3.append(0.0)
            self.imu_t.append(self.imu_t_start + (float(i)/float(num_imu_entries)) * self.imu_t_width)

        # Set up the chosen demonstration.
        if self.demo == 'camera':
            self.camera = Camera('camera1', self.bng, self.vehicle, requested_update_time=0.05, is_using_shared_memory=True, resolution=(1000, 1000), near_far_planes=(0.01, 1000))
        elif demo == 'lidar':
            self.lidar = Lidar('lidar', self.bng, self.vehicle, requested_update_time=0.05, is_using_shared_memory=True, is_visualised=False, vertical_resolution=128, frequency=40)
        elif demo == 'ultrasonic':
            self.us_FL = Ultrasonic('us_FL', self.bng, self.vehicle, requested_update_time=0.05, is_visualised=False, pos=(10.0, -10.0, 0.5), dir=(1.0, -1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_FR = Ultrasonic('us_FR', self.bng, self.vehicle, requested_update_time=0.05, is_visualised=False, pos=(-10.0, -10.0, 0.5), dir=(-1.0, -1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_BL = Ultrasonic('us_BL', self.bng, self.vehicle, requested_update_time=0.05, is_visualised=False, pos=(10.0, 10.0, 0.5), dir=(1.0, 1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_BR = Ultrasonic('us_BR', self.bng, self.vehicle, requested_update_time=0.05, is_visualised=False, pos=(-10.0, 10.0, 0.5), dir=(-1.0, 1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_ML = Ultrasonic('us_ML', self.bng, self.vehicle, requested_update_time=0.05, is_visualised=False, pos=(10.0, 0.0, 0.5), dir=(1.0, 0.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_MR = Ultrasonic('us_MR', self.bng, self.vehicle, requested_update_time=0.05, is_visualised=False, pos=(-10.0, 0.0, 0.5), dir=(-1.0, 0.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
        elif demo == 'radar':
            self.radar = Radar('radar1', self.bng, self.vehicle, requested_update_time=0.05, pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1), resolution=(self.radar_res[0], self.radar_res[1]),
                field_of_view_y=self.radar_fov, near_far_planes=(0.1, self.radar_range_max), range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                range_min_cutoff=0.5, range_direct_max_cutoff=self.radar_range_max)
        elif demo == 'imu_A':
            self.imu1 = AdvancedIMU('imu1', self.bng, self.vehicle, pos=(0.0, 0.0, 0.5), dir=(0, -1, 0), up=(1, 0, 0), gfx_update_time=0.05, physics_update_time=0.0001, is_using_gravity=True, is_visualised=False,
                is_snapping_desired=True, is_force_inside_triangle=True, window_width=1)
        elif demo == 'imu_B':
            pass
        elif demo == 'multi':
            self.lidar = Lidar('lidar', self.bng, self.vehicle, requested_update_time=0.01, is_using_shared_memory=True, is_visualised=False)
        else:
            print("*** WARNING: MAIN SEQUENCE - DEMONSTRATION TITLE NOT FOUND ***")

    def run(self):
        glutIdleFunc(self._update)
        glutMainLoop()

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
        self.focus = [np.cos(rad_yaw) * np.sin(rad_pitch), np.sin(rad_yaw) * np.sin(rad_pitch), np.cos(rad_pitch)]
        self.mouse_x = x
        self.mouse_y = y

    def update_lidar(self, points, vehicle_state):
        assert not self.dirty
        if len(points) == 0:
            return
        self.points = points
        self.points_count = len(points)
        verts = np.array(self.points, dtype=np.float32)
        if self.vertex_buf:
            glDeleteBuffers(1, self.vertex_buf)
        self.vertex_buf = np.uint64(glGenBuffers(1))
        glBindBuffer(GL_ARRAY_BUFFER, self.vertex_buf)
        glBufferData(GL_ARRAY_BUFFER, self.points_count * 4, verts, GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        min_height = points[2::3].min()
        max_height = np.absolute(points[2::3].max() - min_height)
        self.colours[0:self.points_count:3] = points[2::3]
        self.colours[0:self.points_count:3] -= min_height
        self.colours[0:self.points_count:3] /= max_height
        self.colours[1:self.points_count:3] = 0.25
        self.colours[2:self.points_count:3] = 1.0 - self.colours[0:self.points_count:3]
        glDeleteBuffers(1, self.colour_buf)
        self.colour_buf = np.uint64(glGenBuffers(1))
        glBindBuffer(GL_ARRAY_BUFFER, self.colour_buf)
        glBufferData(GL_ARRAY_BUFFER, self.points_count * 4, self.colours, GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        if self.follow and vehicle_state:
            self.focus = vehicle_state['pos']
            self.pos[0] = self.focus[0] + vehicle_state['dir'][0] * -30
            self.pos[1] = self.focus[1] + vehicle_state['dir'][1] * -30
            self.pos[2] = self.focus[2] + vehicle_state['dir'][2] + 10

    def update_ultrasonic(self, d_FL, d_FR, d_BL, d_BR, d_ML, d_MR):
        if d_FL > 5.0:
            self.us_bar_FL = 6
        elif d_FL < 0.5:
            self.us_bar_FL = 0
        else:
            self.us_bar_FL = int(np.floor(d_FL)) + 1
        if d_FR > 5.0:
            self.us_bar_FR = 6
        elif d_FR < 0.5:
            self.us_bar_FR = 0
        else:
            self.us_bar_FR = int(np.floor(d_FR)) + 1
        if d_BL > 5.0:
            self.us_bar_BL = 6
        elif d_BL < 0.5:
            self.us_bar_BL = 0
        else:
            self.us_bar_BL = int(np.floor(d_BL)) + 1
        if d_BR > 5.0:
            self.us_bar_BR = 6
        elif d_BR < 0.5:
            self.us_bar_BR = 0
        else:
            self.us_bar_BR = int(np.floor(d_BR)) + 1
        if d_ML > 5.0:
            self.us_bar_ML = 6
        elif d_ML < 0.5:
            self.us_bar_ML = 0
        else:
            self.us_bar_ML = int(np.floor(d_ML)) + 1
        if d_MR > 5.0:
            self.us_bar_MR = 6
        elif d_MR < 0.5:
            self.us_bar_MR = 0
        else:
            self.us_bar_MR = int(np.floor(d_MR)) + 1

    def _update(self):
        # Handle the update for the chosen demonstration.
        if self.demo == 'camera':
            if self.toggle == 0:
                camera_data = self.camera.poll_shmem_annotation()
                self.camera_image_size = [camera_data[1], camera_data[2]]
                self.camera_image = camera_data[0]
            elif self.toggle == 1:
                camera_data = self.camera.poll_shmem_colour()
                self.camera_image_size = [camera_data[1], camera_data[2]]
                self.camera_image = camera_data[0]
            else:
                camera_data = self.camera.poll_shmem_depth()
                self.camera_image_size = [camera_data[1], camera_data[2]]
                self.camera_image = camera_data[0]

        elif self.demo == 'lidar':
            self.vehicle.sensors.poll()
            points = self.lidar.poll()['pointCloud']
            self.update_lidar(points, self.vehicle.state)

        elif self.demo == 'ultrasonic':
            d_FL, d_FR = self.us_FL.poll()['distance'], self.us_FR.poll()['distance']
            d_BL, d_BR = self.us_BL.poll()['distance'], self.us_BR.poll()['distance']
            d_ML, d_MR = self.us_ML.poll()['distance'], self.us_MR.poll()['distance']
            self.update_ultrasonic(d_FL, d_FR, d_BL, d_BR, d_ML, d_MR)

        elif self.demo == 'radar':
            if self.toggle == 0:
                bscope_data = self.radar.get_bscope_data(range_min=self.radar_range_min, range_max=self.radar_range_max, range_bins=self.radar_bins[0], azimuth_bins=self.radar_bins[1])
                self.radar_image_size = [self.radar_bins[0], self.radar_bins[1]]
                self.radar_image = bscope_data
                ppi_data = self.radar.get_ppi_data(range_min=self.radar_range_min, range_max=self.radar_range_max, range_bins=self.radar_bins[0], azimuth_bins=self.radar_bins[1])
                self.radar_second_image_size = [self.radar_bins[0], self.radar_bins[1]]
                self.radar_second_image = ppi_data
            elif self.toggle == 1:
                bscope_data = self.radar.get_bscope_data(range_min=self.radar_range_min, range_max=self.radar_range_max, range_bins=self.radar_bins[0], azimuth_bins=self.radar_bins[1])
                self.radar_image_size = [self.radar_bins[0], self.radar_bins[1]]
                self.radar_image = bscope_data
            else:
                ppi_data = self.radar.get_ppi_data(range_min=self.radar_range_min, range_max=self.radar_range_max, range_bins=self.radar_bins[0], azimuth_bins=self.radar_bins[1])
                self.radar_image_size = [self.radar_bins[0], self.radar_bins[1]]
                self.radar_image = ppi_data

        elif self.demo == 'imu_A':
            full_imu_data = self.imu1.poll()
            for new_imu_data in full_imu_data:                 # update queues with latest IMU readings (all readings since last poll).
                self.acc1.popleft()
                self.acc1.append(self.imu_acc1_y_min + (max(self.imu_acc_min, min(self.imu_acc_max, new_imu_data['accSmooth'][0])) - self.imu_acc_min) * self.imu_acc1_f)
                self.acc2.popleft()
                self.acc2.append(self.imu_acc2_y_min + (max(self.imu_acc_min, min(self.imu_acc_max, new_imu_data['accSmooth'][1])) - self.imu_acc_min) * self.imu_acc2_f)
                self.acc3.popleft()
                self.acc3.append(self.imu_acc3_y_min + (max(self.imu_acc_min, min(self.imu_acc_max, new_imu_data['accSmooth'][2])) - self.imu_acc_min) * self.imu_acc3_f)
                self.gyro1.popleft()
                self.gyro1.append(new_imu_data['angVelSmooth'][0])
                self.gyro2.popleft()
                self.gyro2.append(new_imu_data['angVelSmooth'][1])
                self.gyro3.popleft()
                self.gyro3.append(new_imu_data['angVelSmooth'][2])
            self.imu_acc1_verts, self.imu_acc2_verts, self.imu_acc3_verts = [], [], [] # reset the vert lists.
            ctr = 0
            for acc1_idx in self.acc1:                                           # create verts from acc1 data.
                self.imu_acc1_verts.append([self.imu_t[ctr], acc1_idx])
                ctr = ctr + 1
            ctr = 0
            for acc2_idx in self.acc2:                                           # create verts from acc2 data.
                self.imu_acc2_verts.append([self.imu_t[ctr], acc2_idx])
                ctr = ctr + 1
            ctr = 0
            for acc3_idx in self.acc3:                                           # create verts from acc3 data.
                self.imu_acc3_verts.append([self.imu_t[ctr], acc3_idx])
                ctr = ctr + 1

        elif self.demo == 'imu_B':
            pass

        elif self.demo == 'multi':
            pass

        else:
            print("*** WARNING: UPDATE - DEMONSTRATION TITLE NOT FOUND ***")

        # Finish the OpenGL flush.
        glutPostRedisplay()

    def _on_display(self):
        glClear(int(GL_COLOR_BUFFER_BIT) | int(GL_DEPTH_BUFFER_BIT))
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        if self.follow:
            gluLookAt(self.pos[0], self.pos[1], self.pos[2], self.focus[0], self.focus[1], self.focus[2], 0.0, 0.0, 1.0)
        else:
            gluLookAt(self.pos[0], self.pos[1], self.pos[2], self.pos[0] + self.focus[0], self.pos[1] + self.focus[1], self.pos[2] + self.focus[2], 0.0, 0.0, 1.0)

        # Individual render data for each demonstration.
        if self.demo == 'camera':
            glViewport(0, 0, self.width, self.height)
            if self.camera_image_size != None:
                if self.toggle == 2:
                    self.render_img(0, 0, self.camera_image, self.camera_image_size[0], self.camera_image_size[1], 1, 1, 1, 2)
                else:
                    self.render_img(0, 0, self.camera_image, self.camera_image_size[0], self.camera_image_size[1], 1, 1, 1, 0)

        elif self.demo == 'lidar':
            glViewport(0, 0, self.width, self.height)
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

        elif self.demo == 'ultrasonic':
            glViewport(0, 0, self.width, self.height)
            self.render_img(153, 150, self.car_img, self.car_img_size[0], self.car_img_size[1], 1, 1, 1, 1)  # The car image.

            # Save and set model view and projection matrix.
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            glOrtho(0, self.width, 0, self.height, -1, 1)
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glLoadIdentity()

            # Render each of the bar sections around the vehicle image.
            glColor3f(0.1, 0.1, 0.1)                                                                        # Top side-bar.
            glRectf(self.loc_ML_0[0], self.loc_ML_0[1], self.loc_ML_0[2], self.loc_ML_0[3])
            glRectf(self.loc_ML_1[0], self.loc_ML_1[1], self.loc_ML_1[2], self.loc_ML_1[3])
            glRectf(self.loc_ML_2[0], self.loc_ML_2[1], self.loc_ML_2[2], self.loc_ML_2[3])
            glRectf(self.loc_ML_3[0], self.loc_ML_3[1], self.loc_ML_3[2], self.loc_ML_3[3])
            glRectf(self.loc_ML_4[0], self.loc_ML_4[1], self.loc_ML_4[2], self.loc_ML_4[3])
            glRectf(self.loc_ML_5[0], self.loc_ML_5[1], self.loc_ML_5[2], self.loc_ML_5[3])
            glRectf(self.loc_ML_6[0], self.loc_ML_6[1], self.loc_ML_6[2], self.loc_ML_6[3])
            if self.us_bar_ML == 0:
                glColor3f(1.0, 0.0, 0.0)
                glRectf(self.loc_ML_0[0], self.loc_ML_0[1], self.loc_ML_0[2], self.loc_ML_0[3])
            elif self.us_bar_ML == 1:
                glColor3f(1.0, 1.0, 0.0)
                glRectf(self.loc_ML_1[0], self.loc_ML_1[1], self.loc_ML_1[2], self.loc_ML_1[3])
            elif self.us_bar_ML == 2:
                glColor3f(1.0, 1.0, 0.0)
                glRectf(self.loc_ML_2[0], self.loc_ML_2[1], self.loc_ML_2[2], self.loc_ML_2[3])
            elif self.us_bar_ML == 3:
                glColor3f(1.0, 1.0, 0.0)
                glRectf(self.loc_ML_3[0], self.loc_ML_3[1], self.loc_ML_3[2], self.loc_ML_3[3])
            elif self.us_bar_ML == 4:
                glColor3f(1.0, 1.0, 1.0)
                glRectf(self.loc_ML_4[0], self.loc_ML_4[1], self.loc_ML_4[2], self.loc_ML_4[3])
            elif self.us_bar_ML == 5:
                glColor3f(1.0, 1.0, 1.0)
                glRectf(self.loc_ML_5[0], self.loc_ML_5[1], self.loc_ML_5[2], self.loc_ML_5[3])
            else:
                glColor3f(1.0, 1.0, 1.0)
                glRectf(self.loc_ML_6[0], self.loc_ML_6[1], self.loc_ML_6[2], self.loc_ML_6[3])
            glColor3f(0.1, 0.1, 0.1)                                                                        # Bottom side-bar.
            glRectf(self.loc_MR_0[0], self.loc_MR_0[1], self.loc_MR_0[2], self.loc_MR_0[3])
            glRectf(self.loc_MR_1[0], self.loc_MR_1[1], self.loc_MR_1[2], self.loc_MR_1[3])
            glRectf(self.loc_MR_2[0], self.loc_MR_2[1], self.loc_MR_2[2], self.loc_MR_2[3])
            glRectf(self.loc_MR_3[0], self.loc_MR_3[1], self.loc_MR_3[2], self.loc_MR_3[3])
            glRectf(self.loc_MR_4[0], self.loc_MR_4[1], self.loc_MR_4[2], self.loc_MR_4[3])
            glRectf(self.loc_MR_5[0], self.loc_MR_5[1], self.loc_MR_5[2], self.loc_MR_5[3])
            glRectf(self.loc_MR_6[0], self.loc_MR_6[1], self.loc_MR_6[2], self.loc_MR_6[3])
            if self.us_bar_MR == 0:
                glColor3f(1.0, 0.0, 0.0)
                glRectf(self.loc_MR_0[0], self.loc_MR_0[1], self.loc_MR_0[2], self.loc_MR_0[3])
            elif self.us_bar_MR == 1:
                glColor3f(1.0, 1.0, 0.0)
                glRectf(self.loc_MR_1[0], self.loc_MR_1[1], self.loc_MR_1[2], self.loc_MR_1[3])
            elif self.us_bar_MR == 2:
                glColor3f(1.0, 1.0, 0.0)
                glRectf(self.loc_MR_2[0], self.loc_MR_2[1], self.loc_MR_2[2], self.loc_MR_2[3])
            elif self.us_bar_MR == 3:
                glColor3f(1.0, 1.0, 0.0)
                glRectf(self.loc_MR_3[0], self.loc_MR_3[1], self.loc_MR_3[2], self.loc_MR_3[3])
            elif self.us_bar_MR == 4:
                glColor3f(1.0, 1.0, 1.0)
                glRectf(self.loc_MR_4[0], self.loc_MR_4[1], self.loc_MR_4[2], self.loc_MR_4[3])
            elif self.us_bar_MR == 5:
                glColor3f(1.0, 1.0, 1.0)
                glRectf(self.loc_MR_5[0], self.loc_MR_5[1], self.loc_MR_5[2], self.loc_MR_5[3])
            else:
                glColor3f(1.0, 1.0, 1.0)
                glRectf(self.loc_MR_6[0], self.loc_MR_6[1], self.loc_MR_6[2], self.loc_MR_6[3])
            for i in range(self.div):
                if self.us_bar_FL == 0:                                                                     # Top-right arc.
                    glColor3f(1.0, 0.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TR_tx0[i], self.TR_ty0[i], self.TR_tx0[i] + self.wid, self.TR_ty0[i] + self.wid)
                if self.us_bar_FL == 1:
                    glColor3f(1.0, 1.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TR_tx1[i], self.TR_ty1[i], self.TR_tx1[i] + self.wid, self.TR_ty1[i] + self.wid)
                if self.us_bar_FL == 2:
                    glColor3f(1.0, 1.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TR_tx2[i], self.TR_ty2[i], self.TR_tx2[i] + self.wid, self.TR_ty2[i] + self.wid)
                if self.us_bar_FL == 3:
                    glColor3f(1.0, 1.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TR_tx3[i], self.TR_ty3[i], self.TR_tx3[i] + self.wid, self.TR_ty3[i] + self.wid)
                if self.us_bar_FL == 4:
                    glColor3f(1.0, 1.0, 1.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TR_tx4[i], self.TR_ty4[i], self.TR_tx4[i] + self.wid, self.TR_ty4[i] + self.wid)
                if self.us_bar_FL == 5:
                    glColor3f(1.0, 1.0, 1.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TR_tx5[i], self.TR_ty5[i], self.TR_tx5[i] + self.wid, self.TR_ty5[i] + self.wid)
                if self.us_bar_FL == 6:
                    glColor3f(1.0, 1.0, 1.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TR_tx6[i], self.TR_ty6[i], self.TR_tx6[i] + self.wid, self.TR_ty6[i] + self.wid)
                if self.us_bar_FR == 0:                                                                     # Bottom-right arc.
                    glColor3f(1.0, 0.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BR_tx0[i], self.BR_ty0[i], self.BR_tx0[i] + self.wid, self.BR_ty0[i] + self.wid)
                if self.us_bar_FR == 1:
                    glColor3f(1.0, 1.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BR_tx1[i], self.BR_ty1[i], self.BR_tx1[i] + self.wid, self.BR_ty1[i] + self.wid)
                if self.us_bar_FR == 2:
                    glColor3f(1.0, 1.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BR_tx2[i], self.BR_ty2[i], self.BR_tx2[i] + self.wid, self.BR_ty2[i] + self.wid)
                if self.us_bar_FR == 3:
                    glColor3f(1.0, 1.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BR_tx3[i], self.BR_ty3[i], self.BR_tx3[i] + self.wid, self.BR_ty3[i] + self.wid)
                if self.us_bar_FR == 4:
                    glColor3f(1.0, 1.0, 1.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BR_tx4[i], self.BR_ty4[i], self.BR_tx4[i] + self.wid, self.BR_ty4[i] + self.wid)
                if self.us_bar_FR == 5:
                    glColor3f(1.0, 1.0, 1.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BR_tx5[i], self.BR_ty5[i], self.BR_tx5[i] + self.wid, self.BR_ty5[i] + self.wid)
                if self.us_bar_FR == 6:
                    glColor3f(1.0, 1.0, 1.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BR_tx6[i], self.BR_ty6[i], self.BR_tx6[i] + self.wid, self.BR_ty6[i] + self.wid)
                if self.us_bar_BL == 0:                                                                     # Top-left arc.
                    glColor3f(1.0, 0.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TL_tx0[i], self.TL_ty0[i], self.TL_tx0[i] + self.wid, self.TL_ty0[i] + self.wid)
                if self.us_bar_BL == 1:
                    glColor3f(1.0, 1.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TL_tx1[i], self.TL_ty1[i], self.TL_tx1[i] + self.wid, self.TL_ty1[i] + self.wid)
                if self.us_bar_BL == 2:
                    glColor3f(1.0, 1.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TL_tx2[i], self.TL_ty2[i], self.TL_tx2[i] + self.wid, self.TL_ty2[i] + self.wid)
                if self.us_bar_BL == 3:
                    glColor3f(1.0, 1.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TL_tx3[i], self.TL_ty3[i], self.TL_tx3[i] + self.wid, self.TL_ty3[i] + self.wid)
                if self.us_bar_BL == 4:
                    glColor3f(1.0, 1.0, 1.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TL_tx4[i], self.TL_ty4[i], self.TL_tx4[i] + self.wid, self.TL_ty4[i] + self.wid)
                if self.us_bar_BL == 5:
                    glColor3f(1.0, 1.0, 1.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TL_tx5[i], self.TL_ty5[i], self.TL_tx5[i] + self.wid, self.TL_ty5[i] + self.wid)
                if self.us_bar_BL == 6:
                    glColor3f(1.0, 1.0, 1.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.TL_tx6[i], self.TL_ty6[i], self.TL_tx6[i] + self.wid, self.TL_ty6[i] + self.wid)
                if self.us_bar_BR == 0:                                                                     # Bottom-left arc.
                    glColor3f(1.0, 0.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BL_tx0[i], self.BL_ty0[i], self.BL_tx0[i] + self.wid, self.BL_ty0[i] + self.wid)
                if self.us_bar_BR == 1:
                    glColor3f(1.0, 1.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BL_tx1[i], self.BL_ty1[i], self.BL_tx1[i] + self.wid, self.BL_ty1[i] + self.wid)
                if self.us_bar_BR == 2:
                    glColor3f(1.0, 1.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BL_tx2[i], self.BL_ty2[i], self.BL_tx2[i] + self.wid, self.BL_ty2[i] + self.wid)
                if self.us_bar_BR == 3:
                    glColor3f(1.0, 1.0, 0.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BL_tx3[i], self.BL_ty3[i], self.BL_tx3[i] + self.wid, self.BL_ty3[i] + self.wid)
                if self.us_bar_BR == 4:
                    glColor3f(1.0, 1.0, 1.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BL_tx4[i], self.BL_ty4[i], self.BL_tx4[i] + self.wid, self.BL_ty4[i] + self.wid)
                if self.us_bar_BR == 5:
                    glColor3f(1.0, 1.0, 1.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BL_tx5[i], self.BL_ty5[i], self.BL_tx5[i] + self.wid, self.BL_ty5[i] + self.wid)
                if self.us_bar_BR == 6:
                    glColor3f(1.0, 1.0, 1.0)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                glRectf(self.BL_tx6[i], self.BL_ty6[i], self.BL_tx6[i] + self.wid, self.BL_ty6[i] + self.wid)

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == 'radar':
            if self.toggle == 0: # both RADAR plots on same screen.
                if self.radar_image_size != None and self.radar_second_image_size != None:
                    glViewport(0, 0, int(self.width / 2), self.height)
                    self.render_img(0, 0, self.radar_image, self.radar_image_size[0], self.radar_image_size[1], 1, 1, 1, 2)
                    glViewport(int(self.width / 2), 0, self.width, self.height)
                    self.render_img(0, 0, self.radar_second_image, self.radar_second_image_size[0], self.radar_second_image_size[1], 1, 1, 1, 2)
            else: # B-scope or PPI plot.
                glViewport(0, 0, self.width, self.height)
                if self.radar_image_size != None:
                    self.render_img(0, 0, self.radar_image, self.radar_image_size[0], self.radar_image_size[1], 1, 1, 1, 2)

        elif self.demo == 'imu_A':
            glViewport(0, 0, self.width, self.height)
            glEnable(GL_LINE_SMOOTH)
            glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

            # Save and set model view and projection matrix.
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            glOrtho(0, self.width, 0, self.height, -1, 1)
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glLoadIdentity()

            # Draw grid.
            glColor3f(0.1, 0.1, 0.1)
            glLineWidth(1.0)
            for i in range(len(self.imu_acc1_grid)):
                self.draw_line(self.imu_acc1_grid[i])
            for i in range(len(self.imu_acc2_grid)):
                self.draw_line(self.imu_acc2_grid[i])
            for i in range(len(self.imu_acc3_grid)):
                self.draw_line(self.imu_acc3_grid[i])

            # Draw lines.
            glColor3f(1.0, 0.0, 0.0)
            glLineWidth(2.0)
            self.draw_line_strip(self.imu_acc1_verts)
            self.draw_line_strip(self.imu_acc2_verts)
            self.draw_line_strip(self.imu_acc3_verts)

            # Draw axes.
            glColor3f(1.0, 1.0, 1.0)
            glLineWidth(3.0)
            self.draw_line(self.imu_acc1_axis_x)
            self.draw_line(self.imu_acc1_axis_y)
            self.draw_line(self.imu_acc2_axis_x)
            self.draw_line(self.imu_acc2_axis_y)
            self.draw_line(self.imu_acc3_axis_x)
            self.draw_line(self.imu_acc3_axis_y)

            glEnable( GL_TEXTURE_2D )
            global texid
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(1.0, 1.0, 1.0)
            glPushMatrix( )
            glTranslate( 700, 500, 0 )
            glPushMatrix( )
            glListBase( base+1 )
            glCallLists( [ord(c) for c in text] )
            glPopMatrix( )
            glPopMatrix( )
            glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == 'imu_B':
            pass

        elif self.demo == 'multi':
           pass

        else:
            print("*** WARNING: MAIN SEQUENCE - DEMONSTRATION TITLE NOT FOUND ***")

        # Flush display - OpenGL.
        glutSwapBuffers()

    def render_img(self, x, y, data, w, h, r, g, b, d_type):
        global bitmap_tex

        # Create texture object.
        if self.bitmap_tex == None:
            self.bitmap_tex = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.bitmap_tex)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        if d_type < 2:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, h, w, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
        else:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, h, w, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, data)

        # Save and set model view and projection matrix.
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.width, 0, self.height, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        # Enable blending.
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_BLEND)

        # Draw textured quad.
        glColor3f(r,g,b)

        width, height = w, h
        if d_type == 1:
            width, height = h, w

        glEnable(GL_TEXTURE_2D)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 1)
        glVertex2f(x, y)
        glTexCoord2f(1, 1)
        glVertex2f(x + width, y)
        glTexCoord2f(1, 0)
        glVertex2f(x + width, y + height)
        glTexCoord2f(0, 0)
        glVertex2f(x, y + height)
        glEnd()
        glDisable(GL_TEXTURE_2D)

        # Restore matrices.
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

        # Disable blending.
        glDisable(GL_BLEND)

    def draw_line_strip(self, vertices):
        glBegin(GL_LINE_STRIP)
        for vertex in vertices:
            glVertex2f(*vertex)
        glEnd()

    def draw_line(self, v):
        glBegin(GL_LINE_STRIP)
        glVertex2f(v[0], v[1])
        glVertex2f(v[2], v[3])
        glEnd()

    def makefont(self, filename, size):
        global texid

        # Load font  and check it is monotype
        face = Face(filename)
        face.set_char_size( size*64 )
        if not face.is_fixed_width:
            raise 'Font is not monotype'

        # Determine largest glyph size
        width, height, ascender, descender = 0, 0, 0, 0
        for c in range(32,128):
            face.load_char( chr(c), FT_LOAD_RENDER | FT_LOAD_FORCE_AUTOHINT )
            bitmap    = face.glyph.bitmap
            width     = max( width, bitmap.width )
            ascender  = max( ascender, face.glyph.bitmap_top )
            descender = max( descender, bitmap.rows-face.glyph.bitmap_top )
        height = ascender+descender

        # Generate texture data
        Z = np.zeros((height*6, width*16), dtype=np.ubyte)
        for j in range(6):
            for i in range(16):
                face.load_char(chr(32+j*16+i), FT_LOAD_RENDER | FT_LOAD_FORCE_AUTOHINT )
                bitmap = face.glyph.bitmap
                x = i*width  + face.glyph.bitmap_left
                y = j*height + ascender - face.glyph.bitmap_top
                Z[y:y+bitmap.rows,x:x+bitmap.width].flat = bitmap.buffer

        # Bound texture
        texid = glGenTextures(1)
        glBindTexture( GL_TEXTURE_2D, texid )
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR )
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR )
        glTexImage2D( GL_TEXTURE_2D, 0, GL_ALPHA, Z.shape[1], Z.shape[0], 0,
                        GL_ALPHA, GL_UNSIGNED_BYTE, Z )

        # Generate display lists
        dx, dy = width/float(Z.shape[1]), height/float(Z.shape[0])
        base = glGenLists(8*16)
        for i in range(8*16):
            c = chr(i)
            x = i%16
            y = i//16-2
            glNewList(base+i, GL_COMPILE)
            if (c == '\n'):
                glPopMatrix( )
                glTranslatef( 0, -height, 0 )
                glPushMatrix( )
            elif (c == '\t'):
                glTranslatef( 4*width, 0, 0 )
            elif (i >= 32):
                glBegin( GL_QUADS )
                glTexCoord2f( (x  )*dx, (y+1)*dy ), glVertex( 0,     -height )
                glTexCoord2f( (x  )*dx, (y  )*dy ), glVertex( 0,     0 )
                glTexCoord2f( (x+1)*dx, (y  )*dy ), glVertex( width, 0 )
                glTexCoord2f( (x+1)*dx, (y+1)*dy ), glVertex( width, -height )
                glEnd( )
                glTranslatef( width, 0, 0 )
            glEndList( )