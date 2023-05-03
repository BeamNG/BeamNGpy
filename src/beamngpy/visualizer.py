import ctypes
from collections import deque
import math
import numpy as np
from PIL import Image
import json
import time
from freetype import *

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from beamngpy.sensors import Camera, Lidar, Ultrasonic, Radar, AdvancedIMU, Mesh
from beamngpy import vec3, Time_Series


CLEAR_COLOUR = (0.1, 0.1, 0.1, 1.0)
MAX_DISTANCE = 120 / 3
MOVE_SPEED = 0.25
TURN_SPEED = 1

base, texid = 0, 0  # for text rendering.

class Visualiser:
    def __init__(self, bng, vehicles, scenario, name, width, height, demo, toggle, map_name):

        # BeamNG state.
        self.bng = bng                                              # The beamng instance.
        self.vehicles = vehicles                                    # The full vehicle dictionary (for all scenarios/modes).
        self.scenario = scenario                                    # The scenario instance.
        self.main_vehicle = vehicles['vehicle_1']                   # The first main vehicle.
        self.map_name = map_name                                    # The name of the current map (string).

        # Demonstration state.
        self.demo = demo                                            # The demo mode (string).
        self.toggle = toggle                                        # For sensor modes, which screen we are currently displaying (each sensor has various toggles.)
        self.is_first_time = True                                   # A flag which indicates if this is the first time we are executing the first iteration.
        self.is_sensor_mode_demo = True                             # A flag which indicates if we are demoing the sensor modes (and not dedicated scenarios).

        # General initialization.
        self.width, self.height = width, height
        self.half_width, self.half_height = int(width * 0.5), int(height * 0.5)
        self.annot_width, self.annot_height = int(width * 0.7), int(height * 0.7)

        # Cached constants.
        self.bitmap_tex = None
        half_pi = math.pi * 0.5
        self.rgb2f = 1.0 / 255.0
        self.v_origin = vec3(0.0, 0.0, 0.0)
        self.v_forward = vec3(0.0, -1.0, 0.0)
        self.v_up = vec3(0.0, 0.0, -1.0)
        self.v_right = vec3(-1.0, 0.0, 0.0)

        # Initialize OpenGL.
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
        glutInitWindowSize(width, height)
        self.window = glutCreateWindow(name)
        if height == 0:
            height = 1
        #glutFullScreen()                           # TODO MAYBE USE THIS LATER ?
        glutDisplayFunc(self._on_display)
        glutKeyboardFunc(self.on_key)
        glutMotionFunc(self.on_drag)
        glutReshapeFunc(self._on_resize)
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
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE)

        # Load font.
        self.makefont('SourceCodePro-Regular.ttf', 16)

        # Initialize storage for all possible sensors.
        self.camera = None
        self.lidar = None
        self.radar = None
        self.us_FL = None
        self.us_FR = None
        self.us_BL = None
        self.us_BR = None
        self.us_ML = None
        self.us_MR = None
        self.imu = None
        self.mesh = None

        # Keys to use for sensor modes/scenarios.
        self.camera_key = b'c'
        self.lidar_key = b'l'
        self.radar_key = b'r'
        self.ultrasonic_key = b'u'
        self.imu_key = b'i'
        self.mesh_key = b'm'
        self.traj_key = b't'
        self.multi_key = b'a'
        self.scenario0_key = b'0'
        self.scenario1_key = b'1'
        self.scenario2_key = b'2'
        self.scenario3_key = b'3'
        self.scenario4_key = b'4'
        self.scenario5_key = b'5'
        self.scenario6_key = b'6'

        # Dedicated scenario target initialization.
        self.scenario2_target = vec3(-798.9409014877656, -408.92386026442546, 103.14585273600176)
        self.scenario2_is_target_hit = False
        self.scenario2_hit_time = 1e12

        # Trajectory initialization.
        self.traj = deque()
        self.traj_memory = 10000
        self.traj_x_min, self.traj_x_max, self.traj_y_min, self.traj_y_max = -1000, 1000, -1000, 1000
        self.pos = []

        # LiDAR initialization.
        self.points_count = 0
        self.points = []
        self.colours = np.zeros(6000000, dtype=np.float32)
        self.focus = [0.0, 0.0, 0.0]
        self.pos = [0.0, 0.0, 0.0]
        self.pitch = 90
        self.yaw = 0
        self.mouse_x = -1
        self.mouse_y = -1
        self.vertex_buf = 0
        self.colour_buf = 0
        self.frame = 1
        self.dirty = False
        self.follow = True

        # Camera initialization.
        self.camera_color_img, self.camera_color_size = [], []
        self.camera_annot_img, self.camera_annot_size = [], []
        self.camera_depth_img, self.camera_depth_size = [], []
        raw_annot_map = bng.camera.get_annotations()                                                                                # annotation colors indexed by name strings.
        self.annot_map = {}
        for key, val in raw_annot_map.items():
            if key == '':                                                                                                           # change the key of the 'empty space' entry.
                self.annot_map['EMPTY_SPACE'] = val
            else:
                self.annot_map[key] = val

        # Ultrasonic initialization.
        car_img = Image.open('car.png')
        self.car_img = np.array(car_img)
        self.car_img_size = [car_img.size[1], car_img.size[0]]                                                                      # NOTE: size vec is flipped for PIL images.
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
        div_f = half_pi / float(self.div)
        self.wid = 15
        cx, cy = 800, 475
        wid_gap = self.wid + gap
        r0, r1, r2, r3, r4, r5, r6 = 190, 190 + wid_gap, 190 + (2 * wid_gap), 190 + (3 * wid_gap), 190 + (4 * wid_gap), 190 + (5 * wid_gap), 190 + (6 * wid_gap)
        self.TR_tx0, self.TR_tx1, self.TR_tx2, self.TR_tx3, self.TR_tx4, self.TR_tx5, self.TR_tx6 = [], [], [], [], [], [], []      # Top-right arc.
        self.TR_ty0, self.TR_ty1, self.TR_ty2, self.TR_ty3, self.TR_ty4, self.TR_ty5, self.TR_ty6 = [], [], [], [], [], [], []
        for i in range(self.div):
            ang = i * div_f
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
            ang = -i * div_f
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
            ang = i * div_f
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
            ang = -i * div_f
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
        car_radar_img = Image.open('car_radar.png')
        self.car_radar_img = np.array(car_radar_img)
        self.car_radar_img_size = [car_radar_img.size[1], car_radar_img.size[0]]
        self.radar_toggle = 0
        self.radar_bscope_img, self.radar_ppi_img, self.radar_rvv_img = [], [], []
        self.radar_bscope_size, self.radar_ppi_size, self.radar_rvv_size = [], [], []
        self.radar_res = [475, 475]             # range, azimuth.
        self.radar_bins = [475, 475, 475]       # range, azimuth, velocity.
        self.radar_fov = 70.0
        self.radar_range_min, self.radar_range_max = 0.1, 100.0
        self.radar_vel_min, self.radar_vel_max = -40.0, 40.0
        fov_azimuth = (self.radar_res[0] / float(self.radar_res[1])) * self.radar_fov
        self.half_fov_azimuth = fov_azimuth * 0.5
        self.max_az_rad = np.deg2rad(fov_azimuth) * 0.5
        self.min_az_rad = -self.max_az_rad
        radar_f1 = self.radar_range_max / (self.radar_bins[0] + 1)
        radar_f2 = -((self.max_az_rad - self.min_az_rad) / (self.radar_bins[1] + 1))
        radius, azimuth = np.mgrid[0.0:self.radar_range_max:radar_f1, self.max_az_rad:self.min_az_rad:radar_f2]
        az_plus_half_pi = azimuth + half_pi
        self.grid_x, self.grid_y = radius * np.cos(az_plus_half_pi), radius * np.sin(az_plus_half_pi)

        # IMU:
        rpy_img = Image.open('imu_rpy.png')
        self.rpy_img = np.array(rpy_img)
        self.rpy_img_size = [rpy_img.size[1], rpy_img.size[0]]                                                      # NOTE: the size is flipped for PIL images.
        self.time_series1, self.time_series2, self.time_series3 = [], [], []
        self.time_series4, self.time_series5, self.time_series6 = [], [], []

        # Mesh sensor:
        self.mesh_data = {}
        self.imu_mass_min, self.imu_mass_max = 0.0, 10.0
        self.mesh_mass_cbar_label0, self.mesh_mass_cbar_label1, self.mesh_mass_cbar_label2 = '0 kg', '5 kg', '10 kg'
        self.imu_force_min, self.imu_force_max = 0.0, 300.0
        self.mesh_force_cbar_label0, self.mesh_force_cbar_label1, self.mesh_force_cbar_label2 = '0 N', '150 N', '300 N'
        self.imu_vel_min, self.imu_vel_max = 0.0, 50.0
        self.mesh_vel_cbar_label0, self.mesh_vel_cbar_label1, self.mesh_vel_cbar_label2 = '0 m/s', '25 m/s', '50 m/s'
        self.imu_stress_min, self.imu_stress_max = -200.0, 200.0
        self.mesh_stress_cbar_label0, self.mesh_stress_cbar_label1, self.mesh_stress_cbar_label2 = '-200 Nm-2', '0 Nm-2', '200 Nm-2'
        self.plan_data, self.elevation_data, self.end_elevation_data = [], [], []
        self.mesh_node_size = 1                                                                                     # The size of node rects on render.
        self.mesh_plan_screen_center, self.mesh_plan_screen_scale = vec3(495, 820), vec3(150, 150)                  # scale/translation for each of the 3 views.
        self.mesh_elev_screen_center, self.mesh_elev_screen_scale = vec3(495, 180), vec3(150, 150)
        self.mesh_end_elev_screen_center, self.mesh_end_elev_screen_scale = vec3(1295, 180), vec3(150, 150)

        # Set up the sensor configuration as chosen by the demo.
        self._set_up_sensors(demo)

    def run(self):
        glutIdleFunc(self._update)
        glutMainLoop()

    def _on_resize(self, width, height):
        if height == 0:
            height = 1
        glViewport(0, 0, width, height)

    def on_key(self, name, *args):

        # Handle the loading of scenarios.
        if self.is_first_time or name == self.scenario0_key:
            self.bng.teleport_vehicle(self.vehicles['vehicle_1'], pos=(-167.36038370872836, 500.836156547889, 75.06553645606348), reset=True)
            self.bng.teleport_vehicle(self.vehicles['vehicle_2'], pos=(-224.4359238577308, 493.73223727132427, 74.97080893622115), reset=True)
            self.bng.teleport_vehicle(self.vehicles['vehicle_3'], pos=(-257.4128802340638, 525.247786108237, 74.93477922917714), reset=True)
            self.bng.teleport_vehicle(self.vehicles['vehicle_4'], pos=(-293.2409596498728, 557.1168029616274, 74.97442921636775), reset=True)
            self.bng.teleport_vehicle(self.vehicles['vehicle_5'], pos=(-285.5863921275886, 526.4913239743473, 74.94484505265063), reset=True)
            self.bng.teleport_vehicle(self.vehicles['vehicle_6'], pos=(-328.90787081420285, 591.2607434741677, 74.9863941054507), reset=True)
            self.bng.teleport_vehicle(self.vehicles['vehicle_7'], pos=(-369.2989675345025, 627.3902712555509, 75.0958857980153), reset=True)
            self.bng.teleport_vehicle(self.vehicles['vehicle_8'], pos=(-364.46489701971586, 633.0149337410967, 75.073325342707), reset=True)
            self.bng.teleport_vehicle(self.vehicles['vehicle_9'], pos=(-342.42810740644927, 630.6641727234382, 74.97903450986632), reset=True)
            self.bng.teleport_vehicle(self.vehicles['vehicle_10'], pos=(-404.3606958804594, 654.6428768548212, 74.97191763509181), reset=True)
            #self.vehicles['vehicle_1'].ai.set_mode('span')
            self.vehicles['vehicle_2'].ai.set_mode('span')
            self.vehicles['vehicle_3'].ai.set_mode('span')
            self.vehicles['vehicle_4'].ai.set_mode('span')
            self.vehicles['vehicle_5'].ai.set_mode('span')
            self.vehicles['vehicle_6'].ai.set_mode('span')
            self.vehicles['vehicle_7'].ai.set_mode('span')
            self.vehicles['vehicle_8'].ai.set_mode('span')
            self.vehicles['vehicle_9'].ai.set_mode('span')
            self.vehicles['vehicle_10'].ai.set_mode('span')
            self.bng.set_relative_camera(pos=(0, -6, 2), dir=(0.0, -1.0, -0.3))
            self.scenario.set_initial_focus('vehicle_1')
            self.is_first_time = False
            self.is_sensor_mode_demo = True
        elif name == self.scenario1_key:
            script_vehicle1, script_vehicle2, script_vehicle3, script_vehicle4, script_vehicle5 = None, None, None, None, None
            script_vehicle6, script_vehicle7, script_vehicle8, script_vehicle9, script_vehicle10 = None, None, None, None, None
            with open('vehicle1.json', 'r') as j:
                script_vehicle1 = json.loads(j.read())
            with open('vehicle2.json', 'r') as j:
                script_vehicle2 = json.loads(j.read())
            with open('vehicle3.json', 'r') as j:
                script_vehicle3 = json.loads(j.read())
            with open('vehicle4.json', 'r') as j:
                script_vehicle4 = json.loads(j.read())
            with open('vehicle5.json', 'r') as j:
                script_vehicle5 = json.loads(j.read())
            with open('vehicle6.json', 'r') as j:
                script_vehicle6 = json.loads(j.read())
            with open('vehicle7.json', 'r') as j:
                script_vehicle7 = json.loads(j.read())
            with open('vehicle8.json', 'r') as j:
                script_vehicle8 = json.loads(j.read())
            with open('vehicle9.json', 'r') as j:
                script_vehicle9 = json.loads(j.read())
            with open('vehicle10.json', 'r') as j:
                script_vehicle10 = json.loads(j.read())
            self.vehicles['vehicle_1'].ai.execute_script(script_vehicle1)
            self.vehicles['vehicle_2'].ai.execute_script(script_vehicle2, 1.875)
            self.vehicles['vehicle_3'].ai.execute_script(script_vehicle3)
            self.vehicles['vehicle_4'].ai.execute_script(script_vehicle4)
            self.vehicles['vehicle_5'].ai.execute_script(script_vehicle5)
            self.vehicles['vehicle_6'].ai.execute_script(script_vehicle6)
            self.vehicles['vehicle_7'].ai.execute_script(script_vehicle7)
            self.vehicles['vehicle_8'].ai.execute_script(script_vehicle8)
            self.vehicles['vehicle_9'].ai.execute_script(script_vehicle9)
            self.vehicles['vehicle_10'].ai.execute_script(script_vehicle10)
            self.bng.set_relative_camera(pos=(0, -7, 2.5), dir=(0.0, -1.0, -0.3))
            self.scenario.set_initial_focus('vehicle_1')
            self.is_sensor_mode_demo = False
            self.demo = '1'
        elif name == self.scenario2_key:
            script_vehicle1, script_vehicle2, script_vehicle3, script_vehicle4, script_vehicle5 = None, None, None, None, None
            script_vehicle6, script_vehicle7, script_vehicle8, script_vehicle9, script_vehicle10 = None, None, None, None, None
            with open('vehicle1_sc2.json', 'r') as j:
                script_vehicle1 = json.loads(j.read())
            with open('vehicle2_sc2.json', 'r') as j:
                script_vehicle2 = json.loads(j.read())
            with open('vehicle3_sc2.json', 'r') as j:
                script_vehicle3 = json.loads(j.read())
            with open('vehicle4_sc2.json', 'r') as j:
                script_vehicle4 = json.loads(j.read())
            with open('vehicle5_sc2.json', 'r') as j:
                script_vehicle5 = json.loads(j.read())
            with open('vehicle6_sc2.json', 'r') as j:
                script_vehicle6 = json.loads(j.read())
            with open('vehicle7_sc2.json', 'r') as j:
                script_vehicle7 = json.loads(j.read())
            with open('vehicle8_sc2.json', 'r') as j:
                script_vehicle8 = json.loads(j.read())
            with open('vehicle9_sc2.json', 'r') as j:
                script_vehicle9 = json.loads(j.read())
            with open('vehicle10_sc2.json', 'r') as j:
                script_vehicle10 = json.loads(j.read())
            self.vehicles['vehicle_1'].ai.execute_script(script_vehicle1, 5.0)
            self.vehicles['vehicle_2'].ai.execute_script(script_vehicle2, 3.0)
            self.vehicles['vehicle_3'].ai.execute_script(script_vehicle3)
            self.vehicles['vehicle_4'].ai.execute_script(script_vehicle4)
            self.vehicles['vehicle_5'].ai.execute_script(script_vehicle5)
            self.vehicles['vehicle_6'].ai.execute_script(script_vehicle6)
            self.vehicles['vehicle_7'].ai.execute_script(script_vehicle7)
            self.vehicles['vehicle_8'].ai.execute_script(script_vehicle8)
            self.vehicles['vehicle_9'].ai.execute_script(script_vehicle9)
            self.vehicles['vehicle_10'].ai.execute_script(script_vehicle10)
            self.bng.set_relative_camera(pos=(0, -8, 2), dir=(0.0, -1.0, -0.2))
            self.scenario.set_initial_focus('vehicle_1')
            self.is_sensor_mode_demo = False
            self.demo = '2'
            self.scenario2_is_target_hit = False
        elif name == self.scenario3_key:
            script_vehicle1, script_vehicle2, script_vehicle3, script_vehicle4, script_vehicle5 = None, None, None, None, None
            script_vehicle6, script_vehicle7, script_vehicle8, script_vehicle9, script_vehicle10 = None, None, None, None, None
            with open('vehicle1_sc3.json', 'r') as j:
                script_vehicle1 = json.loads(j.read())
            with open('vehicle2_sc3.json', 'r') as j:
                script_vehicle2 = json.loads(j.read())
            with open('vehicle3_sc3.json', 'r') as j:
                script_vehicle3 = json.loads(j.read())
            with open('vehicle4_sc3.json', 'r') as j:
                script_vehicle4 = json.loads(j.read())
            with open('vehicle5_sc3.json', 'r') as j:
                script_vehicle5 = json.loads(j.read())
            with open('vehicle6_sc3.json', 'r') as j:
                script_vehicle6 = json.loads(j.read())
            with open('vehicle7_sc3.json', 'r') as j:
                script_vehicle7 = json.loads(j.read())
            with open('vehicle8_sc3.json', 'r') as j:
                script_vehicle8 = json.loads(j.read())
            with open('vehicle9_sc3.json', 'r') as j:
                script_vehicle9 = json.loads(j.read())
            with open('vehicle10_sc3.json', 'r') as j:
                script_vehicle10 = json.loads(j.read())
            self.vehicles['vehicle_1'].ai.execute_script(script_vehicle1)
            self.vehicles['vehicle_2'].ai.execute_script(script_vehicle2)
            self.vehicles['vehicle_3'].ai.execute_script(script_vehicle3, start_delay=5.5)
            self.vehicles['vehicle_4'].ai.execute_script(script_vehicle4)
            self.vehicles['vehicle_5'].ai.execute_script(script_vehicle5, start_delay=9.0)
            self.vehicles['vehicle_6'].ai.execute_script(script_vehicle6)
            self.vehicles['vehicle_7'].ai.execute_script(script_vehicle7)
            self.vehicles['vehicle_8'].ai.execute_script(script_vehicle8)
            self.vehicles['vehicle_9'].ai.execute_script(script_vehicle9)
            self.vehicles['vehicle_10'].ai.execute_script(script_vehicle10)
            self.bng.set_relative_camera(pos=(-10, -8, 5), dir=(-0.7, -1.0, -0.4))
            self.scenario.set_initial_focus('vehicle_1')
            self.is_sensor_mode_demo = False
            self.demo = '3'
        elif name == self.scenario4_key:
            script_vehicle1, script_vehicle2, script_vehicle3, script_vehicle4, script_vehicle5 = None, None, None, None, None
            script_vehicle6, script_vehicle7, script_vehicle8, script_vehicle9, script_vehicle10 = None, None, None, None, None
            with open('vehicle1_sc4.json', 'r') as j:
                script_vehicle1 = json.loads(j.read())
            with open('vehicle2_sc4.json', 'r') as j:
                script_vehicle2 = json.loads(j.read())
            with open('vehicle3_sc4.json', 'r') as j:
                script_vehicle3 = json.loads(j.read())
            with open('vehicle4_sc4.json', 'r') as j:
                script_vehicle4 = json.loads(j.read())
            with open('vehicle5_sc4.json', 'r') as j:
                script_vehicle5 = json.loads(j.read())
            with open('vehicle6_sc4.json', 'r') as j:
                script_vehicle6 = json.loads(j.read())
            with open('vehicle7_sc4.json', 'r') as j:
                script_vehicle7 = json.loads(j.read())
            with open('vehicle8_sc4.json', 'r') as j:
                script_vehicle8 = json.loads(j.read())
            with open('vehicle10_sc4.json', 'r') as j:
                script_vehicle10 = json.loads(j.read())
            self.vehicles['vehicle_1'].ai.execute_script(script_vehicle1, start_delay=3.0)
            self.vehicles['vehicle_2'].ai.execute_script(script_vehicle2)
            self.vehicles['vehicle_3'].ai.execute_script(script_vehicle3)
            self.vehicles['vehicle_4'].ai.execute_script(script_vehicle4, start_delay=3.0)
            self.vehicles['vehicle_5'].ai.execute_script(script_vehicle5)
            self.vehicles['vehicle_6'].ai.execute_script(script_vehicle6, start_delay=7.0)
            self.vehicles['vehicle_7'].ai.execute_script(script_vehicle7)
            self.vehicles['vehicle_8'].ai.execute_script(script_vehicle8)
            self.vehicles['vehicle_10'].ai.execute_script(script_vehicle10)
            self.bng.set_relative_camera(pos=(1, 6, 0.8), dir=(0.1, 1.0, 0.0))
            self.scenario.set_initial_focus('vehicle_1')
            self.is_sensor_mode_demo = False
            self.demo = '4'
        elif name == self.scenario5_key:
            script_vehicle1, script_vehicle2, script_vehicle3, script_vehicle4, script_vehicle5 = None, None, None, None, None
            script_vehicle6, script_vehicle7, script_vehicle8, script_vehicle9, script_vehicle10 = None, None, None, None, None
            with open('vehicle1_sc5.json', 'r') as j:
                script_vehicle1 = json.loads(j.read())
            with open('vehicle2_sc5.json', 'r') as j:
                script_vehicle2 = json.loads(j.read())
            with open('vehicle3_sc5.json', 'r') as j:
                script_vehicle3 = json.loads(j.read())
            with open('vehicle4_sc5.json', 'r') as j:
                script_vehicle4 = json.loads(j.read())
            with open('vehicle5_sc5.json', 'r') as j:
                script_vehicle5 = json.loads(j.read())
            with open('vehicle6_sc5.json', 'r') as j:
                script_vehicle6 = json.loads(j.read())
            with open('vehicle7_sc5.json', 'r') as j:
                script_vehicle7 = json.loads(j.read())
            with open('vehicle9_sc5.json', 'r') as j:
                script_vehicle9 = json.loads(j.read())
            with open('vehicle10_sc5.json', 'r') as j:
                script_vehicle10 = json.loads(j.read())
            self.vehicles['vehicle_1'].ai.execute_script(script_vehicle1, start_delay=1.0)
            self.vehicles['vehicle_2'].ai.execute_script(script_vehicle2)
            self.vehicles['vehicle_3'].ai.execute_script(script_vehicle3)
            self.vehicles['vehicle_4'].ai.execute_script(script_vehicle4, start_delay=2.5)
            self.vehicles['vehicle_5'].ai.execute_script(script_vehicle5)
            self.vehicles['vehicle_6'].ai.execute_script(script_vehicle6, start_delay=11.5)
            self.vehicles['vehicle_7'].ai.execute_script(script_vehicle7)
            self.vehicles['vehicle_9'].ai.execute_script(script_vehicle9, start_delay=6.5)
            self.vehicles['vehicle_10'].ai.execute_script(script_vehicle10)
            self.bng.set_relative_camera(pos=(-4, -6, 3), dir=(-0.7, -1.0, -0.4))
            self.scenario.set_initial_focus('vehicle_1')
            self.is_sensor_mode_demo = False
            self.demo = '5'
        elif name == self.scenario6_key:
            script_vehicle1, script_vehicle2, script_vehicle3, script_vehicle4, script_vehicle5 = None, None, None, None, None
            script_vehicle6, script_vehicle7, script_vehicle8, script_vehicle9, script_vehicle10 = None, None, None, None, None
            with open('vehicle1_sc6.json', 'r') as j:
                script_vehicle1 = json.loads(j.read())
            with open('vehicle2_sc6.json', 'r') as j:
                script_vehicle2 = json.loads(j.read())
            with open('vehicle3_sc6.json', 'r') as j:
                script_vehicle3 = json.loads(j.read())
            with open('vehicle4_sc6.json', 'r') as j:
                script_vehicle4 = json.loads(j.read())
            with open('vehicle5_sc6.json', 'r') as j:
                script_vehicle5 = json.loads(j.read())
            with open('vehicle6_sc6.json', 'r') as j:
                script_vehicle6 = json.loads(j.read())
            with open('vehicle7_sc6.json', 'r') as j:
                script_vehicle7 = json.loads(j.read())
            with open('vehicle8_sc6.json', 'r') as j:
                script_vehicle8 = json.loads(j.read())
            with open('vehicle9_sc6.json', 'r') as j:
                script_vehicle9 = json.loads(j.read())
            with open('vehicle10_sc6.json', 'r') as j:
                script_vehicle10 = json.loads(j.read())
            self.vehicles['vehicle_1'].ai.execute_script(script_vehicle1, start_delay=1.9)
            self.vehicles['vehicle_2'].ai.execute_script(script_vehicle2, start_delay=4.05)
            self.vehicles['vehicle_3'].ai.execute_script(script_vehicle3)
            self.vehicles['vehicle_4'].ai.execute_script(script_vehicle4)
            self.vehicles['vehicle_5'].ai.execute_script(script_vehicle5)
            self.vehicles['vehicle_6'].ai.execute_script(script_vehicle6)
            self.vehicles['vehicle_7'].ai.execute_script(script_vehicle7)
            self.vehicles['vehicle_8'].ai.execute_script(script_vehicle8)
            self.vehicles['vehicle_9'].ai.execute_script(script_vehicle9)
            self.vehicles['vehicle_10'].ai.execute_script(script_vehicle10, start_delay=3.4)
            self.bng.set_relative_camera(pos=(-1, -7, 2), dir=(0, -1.0, -0.4))
            self.scenario.set_initial_focus('vehicle_1')
            self.is_sensor_mode_demo = False
            self.demo = '6'

        if self.is_sensor_mode_demo == False:                                           # Do not check sensor mode buttons if we are running a dedicated scenario.
            return

        # Handle the loading of sensor modes.
        if name == self.camera_key:                                                     # Sensor Mode:  Camera.
            if self.demo == 'camera':
                self.toggle = self.toggle + 1
                if self.toggle > 3:
                    self.toggle = 0
            else:
               self.toggle = 3
               self.demo = 'camera'
               self._set_up_sensors(self.demo)
        elif name == self.lidar_key:                                                    # Sensor Mode:  LiDAR.
            if self.demo != 'lidar':
                self.demo = 'lidar'
                self._set_up_sensors(self.demo)
        elif name == self.radar_key:                                                    # Sensor Mode:  RADAR.
            if self.demo == 'radar':
                self.toggle = self.toggle + 1
                if self.toggle > 1:
                    self.toggle = 0
            else:
                self.toggle = 2
                self.demo = 'radar'
                self._set_up_sensors(self.demo)
        elif name == self.ultrasonic_key:                                               # Sensor Mode:  Ultrasonic.
            if self.demo != 'ultrasonic':
                self.demo = 'ultrasonic'
                self._set_up_sensors(self.demo)
        elif name == self.imu_key:                                                      # Sensor Mode:  IMU.
            if self.demo == 'imu':
                self.toggle = self.toggle + 1
                if self.toggle > 2:
                    self.toggle = 0
            else:
                self.toggle = 2
                self.demo = 'imu'
                self._set_up_sensors(self.demo)
        elif name == self.mesh_key:                                                     # Sensor Mode:  Vehicle Mesh.
            if self.demo == 'mesh':
                self.toggle = self.toggle + 1
                if self.toggle > 3:
                    self.toggle = 0
            else:
                self.toggle = 0
                self.demo = 'mesh'
                self._set_up_sensors(self.demo)
        elif name == self.traj_key:                                                     # Sensor Mode:  Trajectory Plot.
            if self.demo != 'trajectory':
                self.demo = 'trajectory'
                self._set_up_sensors(self.demo)
        elif name == self.multi_key:                                                    # Sensor Mode:  Multi-Sensor.
            if self.demo != 'multi':
                self.demo = 'multi'
                self._set_up_sensors(self.demo)

        # Lidar-specific functionality (for using mouse to move image when not in 'follow' mode).
        if self.demo == 'lidar':
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

    def _set_up_sensors(self, demo):
        # Remove any existing sensors from vehicle.
        if self.camera is not None:
            self.camera.remove()
            self.camera = None
        if self.lidar is not None:
            self.lidar.remove()
            self.lidar = None
        if self.us_FL is not None:
            self.us_FL.remove()
            self.us_FL = None
        if self.us_FR is not None:
            self.us_FR.remove()
            self.us_FR = None
        if self.us_BL is not None:
            self.us_BL.remove()
            self.us_BL = None
        if self.us_BR is not None:
            self.us_BR.remove()
            self.us_BR = None
        if self.us_ML is not None:
            self.us_ML.remove()
            self.us_ML = None
        if self.us_MR is not None:
            self.us_MR.remove()
            self.us_MR = None
        if self.radar is not None:
            self.radar.remove()
            self.radar = None
        if self.imu is not None:
            self.imu.remove()
            self.imu = None
        if self.mesh is not None:
            self.mesh.remove()
            self.mesh = None

        # Set up the chosen demonstration.
        if self.demo == 'camera':
            self.camera = Camera('camera1', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, resolution=(1700, 900), near_far_planes=(0.01, 1000),
                is_streaming=True)

        elif demo == 'lidar':
            self.lidar = Lidar('lidar', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, is_visualised=False, vertical_resolution=128, frequency=40,
                is_streaming=True)

        elif demo == 'ultrasonic':
            self.us_FL = Ultrasonic('us_FL', self.bng, self.main_vehicle, requested_update_time=0.05, is_visualised=False, pos=(10.0, -10.0, 0.5), dir=(1.0, -1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_FR = Ultrasonic('us_FR', self.bng, self.main_vehicle, requested_update_time=0.05, is_visualised=False, pos=(-10.0, -10.0, 0.5), dir=(-1.0, -1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_BL = Ultrasonic('us_BL', self.bng, self.main_vehicle, requested_update_time=0.05, is_visualised=False, pos=(10.0, 10.0, 0.5), dir=(1.0, 1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_BR = Ultrasonic('us_BR', self.bng, self.main_vehicle, requested_update_time=0.05, is_visualised=False, pos=(-10.0, 10.0, 0.5), dir=(-1.0, 1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_ML = Ultrasonic('us_ML', self.bng, self.main_vehicle, requested_update_time=0.05, is_visualised=False, pos=(10.0, 0.0, 0.5), dir=(1.0, 0.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_MR = Ultrasonic('us_MR', self.bng, self.main_vehicle, requested_update_time=0.05, is_visualised=False, pos=(-10.0, 0.0, 0.5), dir=(-1.0, 0.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)

        elif demo == 'radar':
            self.radar = Radar('radar1', self.bng, self.main_vehicle, requested_update_time=0.05, pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1), resolution=(self.radar_res[0], self.radar_res[1]),
                field_of_view_y=self.radar_fov, near_far_planes=(0.1, self.radar_range_max), range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                range_min_cutoff=0.5, range_direct_max_cutoff=self.radar_range_max, range_bins=self.radar_bins[0], azimuth_bins=self.radar_bins[1], vel_bins=self.radar_bins[2],
                is_streaming=True)

        elif demo == 'imu':
            self.imu1 = AdvancedIMU('imu1', self.bng, self.main_vehicle, pos=(0.0, 0.0, 0.5), dir=(0, -1, 0), up=(1, 0, 0), gfx_update_time=0.05, physics_update_time=0.0001, is_using_gravity=True, is_visualised=False,
                is_snapping_desired=True, is_force_inside_triangle=True, window_width=1)
            self.time_series1 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=100.0, y_max=350.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-25.0, data_max=25.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series2 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=400.0, y_max=650.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-25.0, data_max=25.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series3 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=700.0, y_max=950.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-25.0, data_max=25.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series4 = Time_Series(size=10000, x_min=1000.0, x_max=1700, y_min=100.0, y_max=350.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-5.0, data_max=5.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series5 = Time_Series(size=10000, x_min=1000.0, x_max=1700, y_min=400.0, y_max=650.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-5.0, data_max=5.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series6 = Time_Series(size=10000, x_min=1000.0, x_max=1700, y_min=700.0, y_max=950.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-5.0, data_max=5.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)

        elif demo == 'mesh':
            self.mesh = Mesh('mesh', self.bng, self.main_vehicle, gfx_update_time=0.001)

        elif demo == 'multi':    # Camera, LiDAR, RADAR, and Ultrasonic together in one view (four viewports).
            self.camera = Camera('camera1', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, resolution=(850, 450), near_far_planes=(0.01, 1000),
                is_render_annotations=False, is_render_depth=False, is_streaming=True)
            self.lidar = Lidar('lidar', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, is_visualised=False, vertical_resolution=64, frequency=40,
                rays_per_second=1500000, is_streaming=True)
            self.radar = Radar('radar1', self.bng, self.main_vehicle, requested_update_time=0.05, pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1), resolution=(self.radar_res[0], self.radar_res[1]),
                field_of_view_y=self.radar_fov, near_far_planes=(0.1, self.radar_range_max), range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                range_min_cutoff=0.5, range_direct_max_cutoff=self.radar_range_max, range_bins=self.radar_bins[0], azimuth_bins=self.radar_bins[1], vel_bins=self.radar_bins[2],
                is_streaming=True)
            self.us_FL = Ultrasonic('us_FL', self.bng, self.main_vehicle, requested_update_time=0.1, is_visualised=False, pos=(10.0, -10.0, 0.5), dir=(1.0, -1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_FR = Ultrasonic('us_FR', self.bng, self.main_vehicle, requested_update_time=0.1, is_visualised=False, pos=(-10.0, -10.0, 0.5), dir=(-1.0, -1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_BL = Ultrasonic('us_BL', self.bng, self.main_vehicle, requested_update_time=0.1, is_visualised=False, pos=(10.0, 10.0, 0.5), dir=(1.0, 1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_BR = Ultrasonic('us_BR', self.bng, self.main_vehicle, requested_update_time=0.1, is_visualised=False, pos=(-10.0, 10.0, 0.5), dir=(-1.0, 1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_ML = Ultrasonic('us_ML', self.bng, self.main_vehicle, requested_update_time=0.1, is_visualised=False, pos=(10.0, 0.0, 0.5), dir=(1.0, 0.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_MR = Ultrasonic('us_MR', self.bng, self.main_vehicle, requested_update_time=0.1, is_visualised=False, pos=(-10.0, 0.0, 0.5), dir=(-1.0, 0.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)

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

    def _update(self):

        # Trajectory and state sensor poll/update. The trajectory is tracked regardless of mode.
        self.main_vehicle.sensors.poll()                                                                    # poll the state sensor.
        self.pos = self.main_vehicle.state['pos']
        current_pos = [self.pos[0], self.pos[1], self.pos[2]]
        self.traj.append(self.pos)                                                                          # update the trajectory queue.
        if len(self.traj) > self.traj_memory:
            self.traj.popleft()

        # Handle any geometric target logic for the chosen scenario.
        if self.demo == '2':
            p = vec3(current_pos[0], current_pos[1], current_pos[2])
            d = p.distance(self.scenario2_target)
            if self.scenario2_is_target_hit == True:
                dt = time.time() - self.scenario2_hit_time
                if dt > 0.65 and dt < 1.3:
                    self.vehicles['vehicle_1'].control(steering=-1.0, brake=1.0)            # sc2 hit - step 2.
                elif dt >= 1.3 and dt < 5.3:
                    self.vehicles['vehicle_1'].control(steering=0.0)                        # sc2 hit - step 3.
                    self.vehicles['vehicle_1'].ai.set_mode('span')
                elif dt >= 5.3:
                    self.vehicles['vehicle_1'].ai.set_mode('disabled')                      # sc2 hit - step 4.
                    self.vehicles['vehicle_1'].control(steering=0.0, brake=0.3, gear=0)
            if d < 5.0 and self.scenario2_is_target_hit == False:
                self.scenario2_is_target_hit = True
                self.scenario2_hit_time = time.time()
                self.vehicles['vehicle_1'].ai.set_mode('disabled')                          # sc2 hit - step 1.
                self.vehicles['vehicle_1'].control(steering=1.0, brake=1.0)

        # Handle the visualization update for the chosen demonstration.
        if self.demo == 'camera':
            cam_width, cam_height = self.camera.resolution[0], self.camera.resolution[1]
            cam_size = cam_width * cam_height * 4
            if self.toggle == 0 or self.toggle == 3:
                camera_data1 = self.camera.stream_colour(cam_size)
                self.camera_color_size = [cam_width, cam_height]
                self.camera_color_img = camera_data1
            if self.toggle == 1 or self.toggle == 3:
                camera_data2 = self.camera.stream_annotation(cam_size)
                self.camera_annot_size = [cam_width, cam_height]
                self.camera_annot_img = camera_data2
            if self.toggle > 1:
                camera_data3 = self.camera.stream_depth(cam_size)
                self.camera_depth_size = [cam_width, cam_height]
                self.camera_depth_img = camera_data3

        elif self.demo == 'lidar':
            self.main_vehicle.sensors.poll()
            points = self.lidar.stream()
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
            self.colours[0:self.points_count:3] /= max(1e-12, max_height)
            self.colours[1:self.points_count:3] = 0.25
            self.colours[2:self.points_count:3] = 1.0 - self.colours[0:self.points_count:3]
            glDeleteBuffers(1, self.colour_buf)
            self.colour_buf = np.uint64(glGenBuffers(1))
            glBindBuffer(GL_ARRAY_BUFFER, self.colour_buf)
            glBufferData(GL_ARRAY_BUFFER, self.points_count * 4, self.colours, GL_STATIC_DRAW)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
            if self.follow and self.main_vehicle.state:
                self.focus = self.main_vehicle.state['pos']
                self.pos[0] = self.focus[0] + self.main_vehicle.state['dir'][0] * -30
                self.pos[1] = self.focus[1] + self.main_vehicle.state['dir'][1] * -30
                self.pos[2] = self.focus[2] + self.main_vehicle.state['dir'][2] + 10

        elif self.demo == 'ultrasonic':
            d_FL, d_FR = self.us_FL.poll()['distance'], self.us_FR.poll()['distance']
            d_BL, d_BR = self.us_BL.poll()['distance'], self.us_BR.poll()['distance']
            d_ML, d_MR = self.us_ML.poll()['distance'], self.us_MR.poll()['distance']
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

        elif self.demo == 'radar':
            if self.toggle == 0:
                ppi_data = self.radar.stream_ppi()
                self.radar_ppi_size = [self.radar_bins[0], self.radar_bins[1]]
                self.radar_ppi_img = ppi_data
            else:
                rvv_data = self.radar.stream_range_doppler()
                self.radar_rvv_size = [self.radar_bins[0], self.radar_bins[2]]
                self.radar_rvv_img = rvv_data

        elif self.demo == 'imu':
            full_imu_data = self.imu1.poll()
            accX, accY, accZ, gyroX, gyroY, gyroZ = [], [], [], [], [], []
            for i in range(len(full_imu_data)):
                new_imu_data = full_imu_data[i]
                acc, gyro = new_imu_data['accSmooth'], new_imu_data['angVelSmooth']
                accX.append(acc[0])
                accY.append(acc[1])
                accZ.append(acc[2])
                gyroX.append(gyro[0])
                gyroY.append(gyro[1])
                gyroZ.append(gyro[2])
            self.time_series1.update_data(accX)
            self.time_series2.update_data(accY)
            self.time_series3.update_data(accZ)
            self.time_series4.update_data(gyroX)
            self.time_series5.update_data(gyroY)
            self.time_series6.update_data(gyroZ)

        elif self.demo == 'mesh':
            state = self.main_vehicle.state
            if state:
                self.mesh_data = self.mesh.poll()                                           # update the mesh in the mesh class state.
                self.plan_data = self.mesh.project_nodes_to_plane(self.v_origin, self.v_up, self.v_forward, self.mesh_plan_screen_center, self.mesh_plan_screen_scale)
                self.elevation_data = self.mesh.project_nodes_to_plane(self.v_origin, self.v_right, self.v_forward, self.mesh_elev_screen_center, self.mesh_elev_screen_scale)
                self.end_elevation_data = self.mesh.project_nodes_to_plane(self.v_origin, self.v_forward, self.v_right * -1.0, self.mesh_end_elev_screen_center, self.mesh_end_elev_screen_scale)

        elif self.demo == 'multi':
            # Multi: Camera update.
            cam_width, cam_height = self.camera.resolution[0], self.camera.resolution[1]
            camera_data1 = self.camera.stream_colour(cam_width * cam_height * 4)
            self.camera_color_size = [cam_width, cam_height]
            self.camera_color_img = camera_data1

            # Multi: LiDAR update.
            points = self.lidar.stream()
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
            if self.follow and self.vehicle.state:
                self.focus = current_pos
                self.pos[0] = self.focus[0] + self.vehicle.state['dir'][0] * -30
                self.pos[1] = self.focus[1] + self.vehicle.state['dir'][1] * -30
                self.pos[2] = self.focus[2] + self.vehicle.state['dir'][2] + 10

            # Multi: RADAR update.
            ppi_data = self.radar.stream_ppi()
            self.radar_ppi_size = [self.radar_bins[0], self.radar_bins[1]]
            self.radar_ppi_img = ppi_data

            # Multi: Ultrasonic update.
            d_FL, d_FR = self.us_FL.poll()['distance'], self.us_FR.poll()['distance']
            d_BL, d_BR = self.us_BL.poll()['distance'], self.us_BR.poll()['distance']
            d_ML, d_MR = self.us_ML.poll()['distance'], self.us_MR.poll()['distance']
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

        # OpenGL - goes to display function.
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
        if self.demo == 'trajectory':
            if len(self.traj) > 0:
                glViewport(0, 0, self.width, self.height)

                # Save and set model view and projection matrix.
                glMatrixMode(GL_PROJECTION)
                glPushMatrix()
                glLoadIdentity()
                glOrtho(0, self.width, 0, self.height, -1, 1)
                glMatrixMode(GL_MODELVIEW)
                glPushMatrix()
                glLoadIdentity()

                # Plot the trajectory.
                glColor3f(0.7, 0.35, 0.7)
                glLineWidth(2.0)
                x_off, y_off = self.traj[-1][0] - self.half_width, self.traj[-1][1] - self.half_height    # offset to put trajectory starting at center of image.
                traj_lines = len(self.traj) - 1
                for i in range(traj_lines):
                    p0, p1 = self.traj[i], self.traj[i + 1]
                    x0, y0 = p0[0] - x_off, p0[1] - y_off
                    x1, y1 = p1[0] - x_off, p1[1] - y_off
                    self.draw_line([x0, y0, x1, y1])

                glEnable(GL_LINE_SMOOTH)
                glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
                glEnable(GL_BLEND)
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

                # Screen title underline.
                glViewport(0, self.height - 40, self.width, self.height)
                glLineWidth(2.0)
                self.draw_line([40, 928, 300, 930])

                # Draw Text.
                glEnable( GL_TEXTURE_2D )
                glBindTexture( GL_TEXTURE_2D, texid )
                glColor3f(0.85, 0.85, 0.70)
                self.draw_text(50, 950, 'Trajectory')
                glDisable( GL_TEXTURE_2D )

                # Restore matrices.
                glMatrixMode(GL_PROJECTION)
                glPopMatrix()
                glMatrixMode(GL_MODELVIEW)
                glPopMatrix()

        elif self.demo == 'camera':

            # Save and set model view and projection matrix.
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            glOrtho(0, self.width, 0, self.height, -1, 1)
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glLoadIdentity()

            if self.toggle == 0:                                                                                                                # colour image only.
                if len(self.camera_color_size) > 0:
                    glViewport(0, 0, self.width, self.height - 40)
                    self.render_img(50, 50, self.camera_color_img, self.camera_color_size[0], self.camera_color_size[1], 1, 1, 1, 0)

                    # Now deal with the 2D top bar (title etc).
                    glViewport(0, self.height - 40, self.width, self.height)
                    glEnable(GL_LINE_SMOOTH)
                    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
                    glEnable(GL_BLEND)
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

                    # Title underline.
                    glColor3f(0.25, 0.25, 0.15)
                    glLineWidth(2.0)
                    self.draw_line([75, 1, 400, 1])

                    # Draw Text.
                    glEnable( GL_TEXTURE_2D )
                    glBindTexture( GL_TEXTURE_2D, texid )
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(85, 21, 'Camera Sensor - Color Image')
                    self.draw_text(1250, 21, 'Vehicle: ')
                    self.draw_text(1550, 21, 'Model: ')
                    glColor3f(0.85, 0.35, 0.70)
                    self.draw_text(1255, 21, '         ' + self.main_vehicle.vid)
                    self.draw_text(1550, 21, '       ' + self.main_vehicle.model)
                    glDisable( GL_TEXTURE_2D )

            elif self.toggle == 1:                                                                                                              # annotation image only.
                if len(self.camera_annot_size) > 0:
                    glViewport(0, 0, self.annot_width, self.annot_height)
                    self.render_img(50, 50, self.camera_annot_img, self.camera_annot_size[0], self.camera_annot_size[1], 1, 1, 1, 0)

                    # Now deal with the 2D top bar (title etc).
                    glViewport(0, 0, self.width, self.height)
                    glEnable(GL_LINE_SMOOTH)
                    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
                    glEnable(GL_BLEND)
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

                    # Title underline.
                    glColor3f(0.25, 0.25, 0.15)
                    glLineWidth(2.0)
                    self.draw_line([75, 950, 460, 950])

                    # View-division lines.
                    glLineWidth(3.0)
                    self.draw_line([0, self.annot_height, self.annot_width, self.annot_height])
                    self.draw_line([self.annot_width, 0, self.annot_width, self.height])

                    # Draw the colour rectangles for each class key.
                    glLineWidth(1.0)
                    box_x0, box_x1 = 1400, 1420
                    ctr = 0
                    for _, val in self.annot_map.items():
                        y_pos = 881 - (ctr * 26)
                        ctr = ctr + 1
                        box_y0, box_y1 = y_pos - 2, y_pos + 18
                        glColor3f(val[0] * self.rgb2f, val[1] * self.rgb2f, val[2] * self.rgb2f)
                        glRectf(box_x0, box_y0, box_x0 + 20, box_y0 + 20)
                        glColor3f(0.5, 0.5, 0.5)
                        self.draw_line([box_x0, box_y0, box_x1, box_y0])
                        self.draw_line([box_x0, box_y1, box_x1, box_y1])
                        self.draw_line([box_x0, box_y0, box_x0, box_y1])
                        self.draw_line([box_x1, box_y0, box_x1, box_y1])

                    # Draw Text.
                    glEnable( GL_TEXTURE_2D )
                    glBindTexture( GL_TEXTURE_2D, texid )
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(85, 971, 'Camera Sensor - Class Annotations')
                    self.draw_text(85, 875, 'Vehicle: ')
                    self.draw_text(85, 835, 'Model: ')
                    self.draw_text(85, 795, 'Map: ')
                    self.draw_text(1400, 971, 'Color -> Class Map:')
                    glColor3f(0.85, 0.35, 0.70)
                    self.draw_text(85, 875, '         ' + self.main_vehicle.vid)
                    self.draw_text(85, 835, '       ' + self.main_vehicle.model)
                    self.draw_text(85, 795, '     ' + self.map_name)
                    ctr = 0
                    for k in self.annot_map.keys():
                        y_pos = 895 - (ctr * 26)
                        ctr = ctr + 1
                        self.draw_text(1450, y_pos, k)
                    glDisable( GL_TEXTURE_2D )

            elif self.toggle == 2:                                                                                                              # depth image only.
                if len(self.camera_depth_size) > 0:
                    glViewport(0, 0, self.width, self.height)
                    self.render_img(50, 50, self.camera_depth_img, self.camera_depth_size[0], self.camera_depth_size[1], 1, 1, 1, 2)

                    # Now deal with the 2D top bar (title etc).
                    glViewport(0, self.height - 40, self.width, self.height)
                    glEnable(GL_LINE_SMOOTH)
                    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
                    glEnable(GL_BLEND)
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

                    # Title underline.
                    glColor3f(0.25, 0.25, 0.15)
                    glLineWidth(2.0)
                    self.draw_line([75, 1, 400, 1])

                    # Draw Text.
                    glEnable( GL_TEXTURE_2D )
                    glBindTexture( GL_TEXTURE_2D, texid )
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(85, 21, 'Camera Sensor - Depth Image')
                    self.draw_text(1250, 21, 'Vehicle: ')
                    self.draw_text(1550, 21, 'Model: ')
                    glColor3f(0.85, 0.35, 0.70)
                    self.draw_text(1255, 21, '         ' + self.main_vehicle.vid)
                    self.draw_text(1550, 21, '       ' + self.main_vehicle.model)
                    glDisable( GL_TEXTURE_2D )

            else:                                                                                                                               # all images.
                if len(self.camera_color_size) > 0:
                    glViewport(0, 0, self.half_width, self.half_height)
                    self.render_img(50, 80, self.camera_color_img, self.camera_color_size[0], self.camera_color_size[1], 1, 1, 1, 0)
                if len(self.camera_annot_size) > 0:
                    glViewport(self.half_width, 0, self.half_width, self.half_height)
                    self.render_img(50, 80, self.camera_annot_img, self.camera_annot_size[0], self.camera_annot_size[1], 1, 1, 1, 0)
                if len(self.camera_depth_size) > 0:
                    glViewport(0, self.half_height, self.half_width, self.half_height)
                    self.render_img(50, 80, self.camera_depth_img, self.camera_depth_size[0], self.camera_depth_size[1], 1, 1, 1, 2)

                    glViewport(0, 0, self.width, self.height)
                    glEnable(GL_LINE_SMOOTH)
                    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
                    glEnable(GL_BLEND)
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

                    # View-division lines.
                    glColor3f(0.25, 0.25, 0.15)
                    glLineWidth(3.0)
                    self.draw_line([0, self.half_height, self.width, self.half_height])
                    self.draw_line([self.half_width, 0, self.half_width, self.height])

                    # Title underline.
                    glColor3f(0.25, 0.25, 0.15)
                    glLineWidth(2.0)
                    self.draw_line([940, 932, 1105, 932])

                    # Draw Text.
                    glEnable( GL_TEXTURE_2D )
                    glBindTexture( GL_TEXTURE_2D, texid )
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(377, 525, ' Depth Camera')
                    self.draw_text(1230, 25, '  Semantic Annotations')
                    self.draw_text(386, 25, 'Color Camera')
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(950, 950, 'Camera Sensor')
                    self.draw_text(950, 850, 'Vehicle: ')
                    self.draw_text(950, 800, 'Model: ')
                    glColor3f(0.85, 0.35, 0.70)
                    self.draw_text(950, 850, '         ' + self.main_vehicle.vid)
                    self.draw_text(950, 800, '       ' + self.main_vehicle.model)
                    glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == 'lidar':
            glViewport(0, 0, self.width, self.height - 40)

            glEnable(GL_LINE_SMOOTH)
            glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

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

            # Now deal with the 2D top bar (title etc).
            glViewport(0, self.height - 40, self.width, self.height)

            # Save and set model view and projection matrix.
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            glOrtho(0, self.width, 0, self.height, -1, 1)
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glLoadIdentity()

            # Title underline.
            glColor3f(0.25, 0.25, 0.15)
            glLineWidth(2.0)
            self.draw_line([75, 2, 375, 2])

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(85, 20, 'LiDAR Sensor: Point Cloud')
            self.draw_text(1250, 20, 'Vehicle: ')
            self.draw_text(1550, 20, 'Model: ')
            glColor3f(0.85, 0.35, 0.70)
            self.draw_text(1255, 20, '         ' + self.main_vehicle.vid)
            self.draw_text(1550, 20, '       ' + self.main_vehicle.model)
            glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

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

            glEnable(GL_LINE_SMOOTH)
            glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

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

            # Title underline.
            glViewport(0, self.height - 40, self.width, self.height)
            glColor3f(0.25, 0.25, 0.15)
            glLineWidth(2.0)
            self.draw_line([55, 2, 355, 2])

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(85, 20, 'Ultrasonic Sensor x 6')
            self.draw_text(1250, 20, 'Vehicle: ')
            self.draw_text(1550, 20, 'Model: ')
            glColor3f(0.85, 0.35, 0.70)
            self.draw_text(1255, 20, '         ' + self.main_vehicle.vid)
            self.draw_text(1550, 20, '       ' + self.main_vehicle.model)
            glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == 'radar':
            # Save and set model view and projection matrix.
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            glOrtho(0, self.width, 0, self.height, -1, 1)
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glLoadIdentity()

            # Render RADAR images.
            if self.toggle == 0:                                                                                # PPI.
                if len(self.radar_ppi_size) > 0:
                    glViewport(0, 0, self.width, self.height)
                    self.render_img(1160, 10, self.car_radar_img, self.car_radar_img_size[0], self.car_radar_img_size[1], 1, 1, 1, 1)  # From the .png image.
                    glViewport(0, 0, self.width * 2, self.height * 2)
                    self.render_img(100, 12.5, self.radar_ppi_img, self.radar_ppi_size[0], self.radar_ppi_size[1], 1, 1, 1, 0)

                    glEnable(GL_LINE_SMOOTH)
                    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
                    glEnable(GL_BLEND)
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

                    # Draw the PPI scope frame.
                    glViewport(0, 0, self.width, self.height)
                    glLineWidth(2.0)
                    glColor3f(0.3, 0.3, 0.3)
                    self.draw_line([673, 25, 181, 875])                     # left grid line.
                    self.draw_line([673, 25, 1166, 875])                    # right grid line.
                    self.draw_line([673.0, 25.0, 688.0, 16.0])              # grooves.
                    self.draw_line([722.3, 110.0, 737.3, 101.0])
                    self.draw_line([771.6, 195.0, 786.6, 186.0])
                    self.draw_line([820.9, 280.0, 835.9, 271.0])
                    self.draw_line([870.2, 365.0, 885.2, 356.0])
                    self.draw_line([919.5, 450.0, 934.5, 441.0])
                    self.draw_line([968.8, 535.0, 983.8, 526.0])
                    self.draw_line([1018.1, 620.0, 1033.1, 611.0])
                    self.draw_line([1067.4, 705.0, 1082.4, 696.0])
                    self.draw_line([1116.7, 790.0, 1131.7, 781.0])
                    self.draw_line([1166.0, 875.0, 1181.0, 866.0])

                    # Color bar.
                    glLineWidth(3.0)
                    y_min, y_max = self.half_height + 65, self.half_height + 106
                    self.draw_line([69, 49, 115, 49])                       # cb frame - bottom.
                    self.draw_line([69, 451, 115, 451])                     # cb frame - top.
                    self.draw_line([69, 49, 69, 451])                       # cb frame - left.
                    self.draw_line([101, 49, 101, 451])                     # cb frame - right.
                    self.draw_line([100, 250, 115, 250])                    # centreline of colorbar.
                    for i in range(401):                                    # colorbar.
                        r, g, b = 0, 0, 0
                        if i < 200:
                            b = (200 - i) * 0.005
                            g = i * 0.005
                        else:
                            g = (400 - i) * 0.005
                            r = (i - 200) * 0.005
                        glColor3f(r, g, b)
                        y = 50 + i
                        self.draw_line([70, y, 100, y])

                    # Title underline.
                    glViewport(0, self.height - 40, self.width, self.height)
                    glColor3f(0.25, 0.25, 0.15)
                    glLineWidth(2.0)
                    self.draw_line([25, 2, 495, 2])

                    # Draw Text.
                    glEnable( GL_TEXTURE_2D )
                    glBindTexture( GL_TEXTURE_2D, texid )
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(35, 20, 'RADAR Sensor:  Range-Azimuth-Doppler PPI')
                    self.draw_text(1250, 20, 'Vehicle: ')
                    self.draw_text(1550, 20, 'Model: ')
                    glColor3f(0.85, 0.35, 0.70)
                    self.draw_text(1255, 20, '         ' + self.main_vehicle.vid)
                    self.draw_text(1550, 20, '       ' + self.main_vehicle.model)
                    glViewport(0, 0, self.width, self.height)
                    glColor3f(0.4, 0.4, 0.4)
                    self.draw_text(700, 21, '0 m')
                    self.draw_text(745.3, 106.3, '10 m')
                    self.draw_text(796.6, 191.6, '20 m')
                    self.draw_text(843.9, 276.9, '30 m')
                    self.draw_text(893.2, 361.2, '40 m')
                    self.draw_text(942.5, 446.5, '50 m')
                    self.draw_text(991.8, 531.8, '60 m')
                    self.draw_text(1041.1, 616.1, '70 m')
                    self.draw_text(1090.4, 701.4, '80 m')
                    self.draw_text(1140.7, 786.7, '90 m')
                    self.draw_text(1189, 871, '100 m')
                    self.draw_text(125, 60, '0 m/s')
                    self.draw_text(125, 260, '25 m/s')
                    self.draw_text(125, 460, '50 m/s')
                    glColor3f(0.5, 0.5, 0.5)
                    self.draw_text(49, 490, 'Doppler')
                    glDisable( GL_TEXTURE_2D )

            else:
                if len(self.radar_rvv_size) > 0:
                    glViewport(0, 0, self.width * 2, self.height * 2)
                    self.render_img(175, 22.5, self.radar_rvv_img, self.radar_rvv_size[0], self.radar_rvv_size[1], 1, 1, 1, 0)

                    glEnable(GL_LINE_SMOOTH)
                    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
                    glEnable(GL_BLEND)
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

                    # Draw the PPI scope frame.
                    glViewport(0, 0, self.width, self.height)
                    glLineWidth(2.0)
                    glColor3f(0.3, 0.3, 0.3)
                    self.draw_line([350, 45, 1300, 45])                     # bottom frame line.
                    self.draw_line([350, 995, 1300, 995])                   # bottom frame line.
                    self.draw_line([350, 45, 350, 995])                     # left frame line.
                    self.draw_line([1300, 45, 1300, 995])                   # right frame line.
                    div = 95
                    for i in range(11):
                        dv = i * div
                        y = 45 + dv
                        self.draw_line([1300, y, 1310, y])                  # vertical grooves.
                        x = 350 + dv
                        self.draw_line([x, 45, x, 35])                      # horizontal grooves.

                    # Color bar.
                    glLineWidth(3.0)
                    y_min, y_max = self.half_height + 65, self.half_height + 106
                    self.draw_line([69, 49, 115, 49])                       # cb frame - bottom.
                    self.draw_line([69, 451, 115, 451])                     # cb frame - top.
                    self.draw_line([69, 49, 69, 451])                       # cb frame - left.
                    self.draw_line([101, 49, 101, 451])                     # cb frame - right.
                    self.draw_line([100, 250, 115, 250])                    # centreline of colorbar.
                    for i in range(401):                                    # colorbar.
                        r, g, b = 0, 0, 0
                        if i < 200:
                            b = (200 - i) * 0.005
                            g = i * 0.005
                        else:
                            g = (400 - i) * 0.005
                            r = (i - 200) * 0.005
                        glColor3f(r, g, b)
                        y = 50 + i
                        self.draw_line([70, y, 100, y])

                    # Title underline.
                    glViewport(0, self.height - 40, self.width, self.height)
                    glColor3f(0.25, 0.25, 0.15)
                    glLineWidth(2.0)
                    self.draw_line([25, 2, 290, 2])

                    # Draw Text.
                    glEnable( GL_TEXTURE_2D )
                    glBindTexture( GL_TEXTURE_2D, texid )
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(35, 20, 'RADAR: Range - Doppler')
                    self.draw_text(1460, 20, 'Vehicle: ')
                    glColor3f(0.85, 0.35, 0.70)
                    self.draw_text(1465, 20, '         ' + self.main_vehicle.vid)
                    glViewport(0, 0, self.width, self.height)
                    glColor3f(0.4, 0.4, 0.4)
                    txt = ['0 m', '10 m', '20 m', '30 m', '40 m', '50 m', '60 m', '70 m', '80 m', '90 m', '100 m']
                    txt2 = ['-50 m/s', '-40 m/s', '-30 m/s', '-20 m/s', '-10 m/s', '0 m/s', '10 m/s', '20 m/s', '30 m/s', '40 m/s', '50 m/s']
                    for i in range(11):
                        dv = i * div
                        y = 52 + dv
                        self.draw_text(1320, y, txt2[i])                        # vertical text.
                        x = 332 + dv
                        self.draw_text(x, 22, txt[i])                           # horizontal text.
                    self.draw_text(125, 60, '0 m/s')                            # colorbar markers.
                    self.draw_text(125, 260, '25 m/s')
                    self.draw_text(125, 460, '50 m/s')
                    glColor3f(0.5, 0.5, 0.5)
                    self.draw_text(49, 490, 'Doppler')
                    self.draw_text(1420, 525, 'Velocity')
                    self.draw_text(796, 75, 'Range')
                    glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == 'imu':
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
            grid1, grid2, grid3 = self.time_series1.get_grid_lines(), self.time_series2.get_grid_lines(), self.time_series3.get_grid_lines()
            grid4, grid5, grid6 = self.time_series4.get_grid_lines(), self.time_series5.get_grid_lines(), self.time_series6.get_grid_lines()
            glColor3f(0.1, 0.1, 0.1)
            glLineWidth(1.0)
            for i in grid1['thin'] + grid2['thin'] + grid3['thin'] + grid4['thin'] + grid5['thin'] + grid6['thin']:
                self.draw_line(i)
            glColor3f(0.2, 0.2, 0.2)
            glLineWidth(2.0)
            for i in grid1['thick'] + grid2['thick'] + grid3['thick'] + grid4['thick'] + grid5['thick'] + grid6['thick']:
                self.draw_line(i)

            # Draw data polylines.
            glColor3f(1.0, 0.0, 0.0)
            self.draw_line_strip(self.time_series1.get_data_lines())
            self.draw_line_strip(self.time_series2.get_data_lines())
            self.draw_line_strip(self.time_series3.get_data_lines())
            self.draw_line_strip(self.time_series4.get_data_lines())
            self.draw_line_strip(self.time_series5.get_data_lines())
            self.draw_line_strip(self.time_series6.get_data_lines())

            # Draw axes.
            glColor3f(1.0, 1.0, 1.0)
            glLineWidth(3.0)
            ax1, ax2, ax3 = self.time_series1.get_axes_lines(), self.time_series2.get_axes_lines(), self.time_series3.get_axes_lines()
            ax4, ax5, ax6 = self.time_series4.get_axes_lines(), self.time_series5.get_axes_lines(), self.time_series6.get_axes_lines()
            self.draw_line(ax1[0])
            self.draw_line(ax1[1])
            self.draw_line(ax2[0])
            self.draw_line(ax2[1])
            self.draw_line(ax3[0])
            self.draw_line(ax3[1])
            self.draw_line(ax4[0])
            self.draw_line(ax4[1])
            self.draw_line(ax5[0])
            self.draw_line(ax5[1])
            self.draw_line(ax6[0])
            self.draw_line(ax6[1])

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.65, 0.65, 0.3)
            self.draw_text(1710, 275, 'X [roll]')
            self.draw_text(1710, 570, 'Y [pitch]')
            self.draw_text(1710, 875, 'Z [yaw]')
            self.draw_text(350, 985, 'IMU - TRI-AXIAL ACCELERATION')
            self.draw_text(1250, 985, ' IMU - TRI-AXIAL GYROSCOPIC')
            self.draw_text(1300, 45, 'time (seconds)')
            self.draw_text(400, 45, 'time (seconds)')
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(790, 80, '0s')
            self.draw_text(690, 80, '-1s')
            self.draw_text(540, 80, '-2s')
            self.draw_text(390, 80, '-3s')
            self.draw_text(240, 80, '-4s')
            self.draw_text(90, 80, '-5s')
            self.draw_text(44, 231, '0')
            self.draw_text(44, 531, '0')
            self.draw_text(44, 831, '0')
            self.draw_text(20, 107, '-50.0')
            self.draw_text(30, 357, '50.0')
            self.draw_text(20, 408, '-50.0')
            self.draw_text(30, 657, '50.0')
            self.draw_text(20, 708, '-50.0')
            self.draw_text(30, 957, '50.0')
            self.draw_text(1690, 80, '0s')
            self.draw_text(1590, 80, '-1s')
            self.draw_text(1440, 80, '-2s')
            self.draw_text(1290, 80, '-3s')
            self.draw_text(1140, 80, '-4s')
            self.draw_text(990, 80, '-5s')
            self.draw_text(944, 231, '0')
            self.draw_text(944, 531, '0')
            self.draw_text(944, 831, '0')
            self.draw_text(920, 107, '-5.0')
            self.draw_text(930, 357, '5.0')
            self.draw_text(920, 408, '-5.0')
            self.draw_text(930, 657, '5.0')
            self.draw_text(920, 708, '-5.0')
            self.draw_text(930, 957, '5.0')
            glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == 'mesh':
            if len(self.plan_data) > 0:
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

                # Draw beams (1 viewport with different sections for each projection of mesh).
                glLineWidth(1.0)
                if self.toggle == 0:                                                                                # mass distribution.
                    mesh_data = self.mesh_data['nodes']
                    lines = self.plan_data[1] + self.elevation_data[1] + self.end_elevation_data[1]
                    num_lines = len(lines)
                    for i in range(num_lines):
                        node1, node2 = lines[i][2][0], lines[i][2][1]
                        mass1, mass2 = mesh_data[node1]['mass'], mesh_data[node2]['mass']
                        avg_mass = (mass1 + mass2) * 0.5
                        self.set_mesh_color(avg_mass, self.imu_mass_min, self.imu_mass_max)
                        p1, p2 = lines[i][0], lines[i][1]
                        self.draw_line([p1[0], p1[1], p2[0], p2[1]])

                elif self.toggle == 1:                                                                              # force distribution.
                    mesh_data = self.mesh_data['nodes']
                    lines = self.plan_data[1] + self.elevation_data[1] + self.end_elevation_data[1]
                    num_lines = len(lines)
                    for i in range(num_lines):
                        node1, node2 = lines[i][2][0], lines[i][2][1]
                        md1, md2 = mesh_data[node1]['force'], mesh_data[node2]['force']
                        f_mag_1 = vec3(md1['x'], md1['y'], md1['z']).length()
                        f_mag_2 = vec3(md2['x'], md2['y'], md2['z']).length()
                        avg_f = (f_mag_1 + f_mag_2) * 0.5
                        self.set_mesh_color(avg_f, self.imu_force_min, self.imu_force_max)
                        p1, p2 = lines[i][0], lines[i][1]
                        self.draw_line([p1[0], p1[1], p2[0], p2[1]])

                elif self.toggle == 2:                                                                              # velocity distribution.
                    mesh_data = self.mesh_data['nodes']
                    lines = self.plan_data[1] + self.elevation_data[1] + self.end_elevation_data[1]
                    num_lines = len(lines)
                    for i in range(num_lines):
                        node1, node2 = lines[i][2][0], lines[i][2][1]
                        md1, md2 = mesh_data[node1]['vel'], mesh_data[node2]['vel']
                        v_mag_1 = vec3(md1['x'], md1['y'], md1['z']).length()
                        v_mag_2 = vec3(md2['x'], md2['y'], md2['z']).length()
                        avg_v = (v_mag_1 + v_mag_2) * 0.5
                        self.set_mesh_color(avg_v, self.imu_vel_min, self.imu_vel_max)
                        p1, p2 = lines[i][0], lines[i][1]
                        self.draw_line([p1[0], p1[1], p2[0], p2[1]])

                else:                                                                                               # beam stresses.
                    stresses = self.mesh_data['beams']
                    lines = self.plan_data[1]
                    num_lines = len(lines)
                    for i in range(num_lines):
                        self.set_mesh_color(stresses[i]['stress_norm'], self.imu_stress_min, self.imu_stress_max)
                        p1, p2 = lines[i][0], lines[i][1]
                        self.draw_line([p1[0], p1[1], p2[0], p2[1]])
                    lines = self.elevation_data[1]
                    for i in range(num_lines):
                        self.set_mesh_color(stresses[i]['stress_norm'], self.imu_stress_min, self.imu_stress_max)
                        p1, p2 = lines[i][0], lines[i][1]
                        self.draw_line([p1[0], p1[1], p2[0], p2[1]])
                    lines = self.end_elevation_data[1]
                    for i in range(num_lines):
                        self.set_mesh_color(stresses[i]['stress_norm'], self.imu_stress_min, self.imu_stress_max)
                        p1, p2 = lines[i][0], lines[i][1]
                        self.draw_line([p1[0], p1[1], p2[0], p2[1]])

                # Draw nodes.
                lines = self.elevation_data[1]
                nodes = self.plan_data[0] + self.elevation_data[0] + self.end_elevation_data[0]
                num_nodes = len(nodes)
                node_size = self.mesh_node_size
                glColor3f(0.75, 0.75, 0.60)
                for i in range(num_nodes):
                    node = nodes[i]
                    x, y = node[0], node[1]
                    glRectf(x - node_size, y - node_size, x + node_size, y + node_size)

                # View-division lines.
                glColor3f(0.25, 0.25, 0.15)
                glLineWidth(3.0)
                self.draw_line([0, self.half_height, self.width, self.half_height])
                self.draw_line([self.half_width, 0, self.half_width, self.height])

                # Screen title underline.
                glLineWidth(2.0)
                self.draw_line([940, 928, 1289, 930])

                # Color bar.
                glLineWidth(3.0)
                y_min, y_max = self.half_height + 65, self.half_height + 106
                self.draw_line([999, y_min - 1, 1701, y_min - 1])       # cb frame - bottom.
                self.draw_line([999, y_max + 1, 1701, y_max + 1])       # cb frame - top.
                self.draw_line([999, y_min - 15, 999, y_max + 1])       # cb frame - left.
                self.draw_line([1701, y_min - 15, 1701, y_max + 1])     # cb frame - right.
                self.draw_line([1350, y_min - 15, 1350, y_max + 1])     # centreline of colorbar.
                for x in range(1000, 1700):                             # colorbar.
                    self.set_mesh_color(x, 1000, 1700)
                    self.draw_line([x, y_min, x, y_max])

                # Draw Text.
                glEnable( GL_TEXTURE_2D )
                glBindTexture( GL_TEXTURE_2D, texid )
                glColor3f(0.85, 0.85, 0.70)
                if self.toggle == 0:
                    self.draw_text(950, 950, 'VEHICLE MASS DISTRIBUTION')
                    self.draw_text(983, y_min - 32, self.mesh_mass_cbar_label0)
                    self.draw_text(1332, y_min - 32, self.mesh_mass_cbar_label1)
                    self.draw_text(1677, y_min - 32, self.mesh_mass_cbar_label2)

                elif self.toggle == 1:
                    self.draw_text(950, 950, 'VEHICLE FORCE DISTRIBUTION')
                    self.draw_text(983, y_min - 32, self.mesh_force_cbar_label0)
                    self.draw_text(1332, y_min - 32, self.mesh_force_cbar_label1)
                    self.draw_text(1677, y_min - 32, self.mesh_force_cbar_label2)

                elif self.toggle == 2:
                    self.draw_text(950, 950, 'VEHICLE VELOCITY DISTRIBUTION')
                    self.draw_text(980, y_min - 32, self.mesh_vel_cbar_label0)
                    self.draw_text(1327, y_min - 32, self.mesh_vel_cbar_label1)
                    self.draw_text(1670, y_min - 32, self.mesh_vel_cbar_label2)

                else:
                    self.draw_text(950, 950, 'VEHICLE STRESS DISTRIBUTION')
                    self.draw_text(980, y_min - 32, self.mesh_stress_cbar_label0)
                    self.draw_text(1327, y_min - 32, self.mesh_stress_cbar_label1)
                    self.draw_text(1670, y_min - 32, self.mesh_stress_cbar_label2)

                glColor3f(0.85, 0.85, 0.70)
                self.draw_text(422, 530, ' Top')
                self.draw_text(392, 30, '  Right')
                self.draw_text(1280, 30, '    Front')
                self.draw_text(950, 880, 'Vehicle: ')
                self.draw_text(950, 845, 'Model: ')
                glColor3f(0.85, 0.35, 0.70)
                self.draw_text(950, 880, '         ' + self.main_vehicle.vid)
                self.draw_text(950, 845, '       ' + self.main_vehicle.model)
                glDisable( GL_TEXTURE_2D )

                # Restore matrices.
                glMatrixMode(GL_PROJECTION)
                glPopMatrix()
                glMatrixMode(GL_MODELVIEW)
                glPopMatrix()

        elif self.demo == 'multi':

            if len(self.camera_color_size) > 0:     # camera - color image.
                glViewport(0, self.half_height, self.half_width * 2, self.half_height * 2)	# TL
                self.render_img(25, 25, self.camera_color_img, self.camera_color_size[0], self.camera_color_size[1], 1, 1, 1, 0)

            glViewport(self.half_width, 0, self.half_width, self.half_height)
            self.render_img(153, 150, self.car_img, self.car_img_size[0], self.car_img_size[1], 1, 1, 1, 1)  # The ultrasonic .png image.

            if len(self.radar_ppi_size) > 0:
                glViewport(self.half_width, self.half_height, self.half_width, self.half_height)
                self.render_img(1160, 10, self.car_radar_img, self.car_radar_img_size[0], self.car_radar_img_size[1], 1, 1, 1, 1)  # From RADAR .png image.
                glViewport(self.half_width, self.half_height, self.half_width * 2, self.half_height * 2)
                self.render_img(100, 12.5, self.radar_ppi_img, self.radar_ppi_size[0], self.radar_ppi_size[1], 1, 1, 1, 0)

            glEnable(GL_LINE_SMOOTH)
            glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            if self.points_count > 0:
                glViewport(0, 0, self.half_width, self.half_height)
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

            # Save and set model view and projection matrix.
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            glOrtho(0, self.width, 0, self.height, -1, 1)
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glLoadIdentity()

            if len(self.radar_ppi_size) > 0:
                glViewport(self.half_width, self.half_height, self.half_width, self.half_height)

                # Draw the PPI scope frame.
                glLineWidth(2.0)
                glColor3f(0.3, 0.3, 0.3)
                self.draw_line([673, 25, 181, 875])                     # left grid line.
                self.draw_line([673, 25, 1166, 875])                    # right grid line.
                self.draw_line([673.0, 25.0, 688.0, 16.0])              # grooves.
                self.draw_line([722.3, 110.0, 737.3, 101.0])
                self.draw_line([771.6, 195.0, 786.6, 186.0])
                self.draw_line([820.9, 280.0, 835.9, 271.0])
                self.draw_line([870.2, 365.0, 885.2, 356.0])
                self.draw_line([919.5, 450.0, 934.5, 441.0])
                self.draw_line([968.8, 535.0, 983.8, 526.0])
                self.draw_line([1018.1, 620.0, 1033.1, 611.0])
                self.draw_line([1067.4, 705.0, 1082.4, 696.0])
                self.draw_line([1116.7, 790.0, 1131.7, 781.0])
                self.draw_line([1166.0, 875.0, 1181.0, 866.0])

                # Color bar.
                glLineWidth(3.0)
                y_min, y_max = self.half_height + 65, self.half_height + 106
                self.draw_line([69, 49, 115, 49])                       # cb frame - bottom.
                self.draw_line([69, 451, 115, 451])                     # cb frame - top.
                self.draw_line([69, 49, 69, 451])                       # cb frame - left.
                self.draw_line([101, 49, 101, 451])                     # cb frame - right.
                self.draw_line([100, 250, 115, 250])                    # centreline of colorbar.
                for i in range(401):                                    # colorbar.
                    r, g, b = 0, 0, 0
                    if i < 200:
                        b = (200 - i) * 0.005
                        g = i * 0.005
                    else:
                        g = (400 - i) * 0.005
                        r = (i - 200) * 0.005
                    glColor3f(r, g, b)
                    y = 50 + i
                    self.draw_line([70, y, 100, y])

            # Ultrasonic.
            glViewport(self.half_width, 0, self.half_width, self.half_height)
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

            # View-division lines.
            glViewport(0, 0, self.width, self.height)
            glColor3f(0.25, 0.25, 0.15)
            glLineWidth(3.0)
            self.draw_line([0, self.half_height, self.width, self.half_height])
            self.draw_line([self.half_width, 0, self.half_width, self.height])

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(400, 530, ' Camera')
            self.draw_text(400, 30, '  LiDAR')
            self.draw_text(1255, 30, '    Ultrasonic')
            self.draw_text(1280, 530, '    RADAR')
            self.draw_text(1555, 60, 'Vehicle: ')
            self.draw_text(1580, 30, 'Model: ')
            glColor3f(0.85, 0.35, 0.70)
            self.draw_text(1555, 60, '         ' + self.main_vehicle.vid)
            self.draw_text(1580, 30, '       ' + self.main_vehicle.model)
            glViewport(self.half_width, self.half_height, self.half_width, self.half_height)
            glColor3f(0.4, 0.4, 0.4)
            self.draw_text(700, 21, '0 m')
            self.draw_text(745.3, 106.3, '10 m')
            self.draw_text(796.6, 191.6, '20 m')
            self.draw_text(843.9, 276.9, '30 m')
            self.draw_text(893.2, 361.2, '40 m')
            self.draw_text(942.5, 446.5, '50 m')
            self.draw_text(991.8, 531.8, '60 m')
            self.draw_text(1041.1, 616.1, '70 m')
            self.draw_text(1090.4, 701.4, '80 m')
            self.draw_text(1140.7, 786.7, '90 m')
            self.draw_text(1189, 871, '100 m')
            self.draw_text(125, 60, '0 m/s')
            self.draw_text(125, 260, '25 m/s')
            self.draw_text(125, 460, '50 m/s')
            glColor3f(0.5, 0.5, 0.5)
            self.draw_text(49, 490, 'Doppler')
            glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        # Final tidy up for frame data, before going to next update/display.
        self.radar_ppi_size, self.radar_bscope_size, self.radar_rvv_size = [], [], []

        # Flush display - OpenGL.
        glutSwapBuffers()

    def render_img(self, x, y, data, h, w, r, g, b, d_type):
        global bitmap_tex

        # Create texture object.
        if self.bitmap_tex == None:
            self.bitmap_tex = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.bitmap_tex)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        ht, wd = h, w
        if d_type == 0:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, h, w, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
        elif d_type == 1:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
            ht, wd = w, h
        else:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, h, w, 0, GL_DEPTH_COMPONENT, GL_FLOAT, data)

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
        glColor3f(r, g, b)

        glEnable(GL_TEXTURE_2D)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 1)
        glVertex2f(x, y)
        glTexCoord2f(1, 1)
        glVertex2f(x + ht, y)
        glTexCoord2f(1, 0)
        glVertex2f(x + ht, y + wd)
        glTexCoord2f(0, 0)
        glVertex2f(x, y + wd)
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

    def draw_text(self, x, y, txt):
        glPushMatrix()
        glTranslate(x, y, 0)
        glPushMatrix( )
        glListBase(base + 1)
        glCallLists([ord(c) for c in txt])
        glPopMatrix()
        glPopMatrix()

    def set_mesh_color(self, val, min, max):
        ratio = (val - min) / (max - min)
        glColor3f(ratio, 0.0, 1.0 - ratio)

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
        base = glGenLists(8 * 16)
        for i in range(8 * 16):
            c = chr(i)
            x = i % 16
            y = i // 16-2
            glNewList(base + i, GL_COMPILE)
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