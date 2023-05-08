import ctypes
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
from beamngpy import vec3, Time_Series, Mesh_View, US_View, Trajectory

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
        self.is_ai_driving = True                                   # A flag which indicates if the main vehicle is to be AI-driven (True) or human driven (False).
        self.is_final_display = False                               # A flag which indicates if we are in the final (frozen) display phase of a scenario demo.

        # General initialization.
        self.width, self.height = width, height
        self.half_width, self.half_height = int(width * 0.5), int(height * 0.5)
        self.annot_width, self.annot_height = int(width * 0.7), int(height * 0.7)

        # Cached constants.
        self.bitmap_tex = None
        half_pi = math.pi * 0.5
        self.rgb2f = 1.0 / 255.0

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
        self.camera2 = None
        self.lidar = None
        self.radar = None
        self.us_FL = None
        self.us_FR = None
        self.us_BL = None
        self.us_BR = None
        self.us_ML = None
        self.us_MR = None
        self.imu1 = None
        self.imu2 = None
        self.mesh = None

        # Keys to use for sensor modes/scenarios.
        self.camera_key = b'c'                                                                          # Sensor mode keys.
        self.lidar_key = b'l'
        self.radar_key = b'r'
        self.ultrasonic_key = b'u'
        self.imu_key = b'i'
        self.mesh_key = b'm'
        self.traj_key = b't'
        self.multi_key = b'a'
        self.scenario0_key = b'0'                                                                       # Dedicated scenario keys.
        self.scenario1_key = b'1'
        self.scenario2_key = b'2'
        self.scenario3_key = b'3'
        self.scenario4_key = b'4'
        self.scenario5_key = b'5'
        self.scenario6_key = b'6'
        self.ai_key = b' '                                                                              # Switch between AI-driven and human-driven.

        # Dedicated scenario target initialization.
        self.scenario1_target = vec3(724.2028016672593, -886.0135850848401, 149.6246404944941)          # scenario 1.
        self.scenario1_is_target_hit = False
        self.scenario1_hit_time = 1e12
        self.scenario2_target = vec3(-798.9409014877656, -408.92386026442546, 103.14585273600176)       # scenario 2.
        self.scenario2_is_target_hit = False
        self.scenario2_hit_time = 1e12
        self.scenario3_target = vec3(-87.92972193099558, -677.5602118819952, 140.7823222070001)         # scenario 3.
        self.is_scenario_3_paused = False
        self.scenario4_target = vec3(-349.4291180372238, 469.7554428577423, 89.0815799459815)           # scenario 4.
        self.scenario4_is_target_hit = False
        self.scenario4_hit_time = 1e12
        self.is_scenario_4_paused = False
        self.scenario5_target = vec3(-431.46150547266006, -761.1655665487051, 141.8720615557395)        # scenario 5.
        self.scenario5_is_target_hit = False
        self.scenario5_hit_time = 1e12
        self.is_scenario_5_paused = False
        self.scenario6_target = vec3(-972.4352, -636.335896, 106.9540274)                             # scenario 6.
        self.scenario6_is_target_hit = False
        self.scenario6_hit_time = 1e12
        self.is_scenario_6_paused = False

        # RGB colorbar.
        rgb_colorbar_img = Image.open('rgb_colorbar.png')
        self.rgb_colorbar = np.array(rgb_colorbar_img)
        self.rgb_colorbar_size = [rgb_colorbar_img.size[1], rgb_colorbar_img.size[0]]

        # Trajectory initialization.
        self.trajectory_view = None

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
        self.camera2_color_img, self.camera2_color_size = [], []
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
        self.us0, self.us1, self.us2, self.us3, self.us4, self.us5 = None, None, None, None, None, None
        self.us_view = []
        car_img = Image.open('car.png')
        self.car_img = np.array(car_img)
        self.car_img_size = [car_img.size[1], car_img.size[0]]                                                                      # NOTE: size vec is flipped for PIL images.

        # RADAR initialization.
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
        self.imu1 = None
        self.time_series1, self.time_series2, self.time_series3 = None, None, None
        self.time_series4, self.time_series5, self.time_series6 = None, None, None

        # Mesh sensor:
        self.mesh = None
        self.mesh_view = []

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
        if name == self.ai_key:
            if self.is_ai_driving == True:
                self.vehicles['vehicle_1'].ai.set_mode('disabled')
            else:
                self.vehicles['vehicle_1'].ai.set_mode('span')
            self.is_ai_driving = not self.is_ai_driving

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
            self.vehicles['vehicle_1'].ai.set_mode('span')
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
            self.demo = 0
            self.is_final_display = False
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
            self._set_up_sensors(self.demo)
            self.scenario1_is_target_hit = False
            self.is_final_display = False
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
            self.bng.teleport_vehicle(self.vehicles['vehicle_11'], pos=(-772.6274979914087, -387.2534139925265, 103.49886869300644), reset=True) # Move c-box to original pos.
            self.bng.set_relative_camera(pos=(0, -8, 2), dir=(0.0, -1.0, -0.2))
            self.scenario.set_initial_focus('vehicle_1')
            self.is_sensor_mode_demo = False
            self.demo = '2'
            self._set_up_sensors(self.demo)
            self.scenario2_is_target_hit = False
            self.is_final_display = False
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
            self._set_up_sensors(self.demo)
            self.is_scenario_3_paused = False
            self.is_final_display = False
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
            self._set_up_sensors(self.demo)
            self.scenario4_is_target_hit = False
            self.is_scenario_4_paused = False
            self.is_final_display = False
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
            self._set_up_sensors(self.demo)
            self.scenario5_is_target_hit = False
            self.is_scenario_5_paused = False
            self.is_final_display = False
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
            self.vehicles['vehicle_1'].ai.execute_script(script_vehicle1, start_delay=1.9)
            self.vehicles['vehicle_2'].ai.execute_script(script_vehicle2)
            self.vehicles['vehicle_3'].ai.execute_script(script_vehicle3)
            self.vehicles['vehicle_4'].ai.execute_script(script_vehicle4)
            self.vehicles['vehicle_5'].ai.execute_script(script_vehicle5)
            self.vehicles['vehicle_6'].ai.execute_script(script_vehicle6)
            self.bng.set_relative_camera(pos=(-1, -7, 2), dir=(0, -1.0, -0.4))
            self.scenario.set_initial_focus('vehicle_1')
            self.is_sensor_mode_demo = False
            self.demo = '6'
            self._set_up_sensors(self.demo)
            self.scenario6_is_target_hit = False
            self.is_scenario_6_paused = False
            self.is_final_display = False

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
            if self.demo != 'imu':
                self.demo = 'imu'
                self._set_up_sensors(self.demo)
        elif name == self.mesh_key:                                                     # Sensor Mode:  Vehicle Mesh.
            if self.demo == 'mesh':
                self.toggle = self.toggle + 1
                if self.toggle > 4:
                    self.toggle = 0
            else:
                self.toggle = 0
                self.demo = 'mesh'
                self._set_up_sensors(self.demo)
            if self.toggle == 0:
                self.mesh_view.change_mode('mass')
            elif self.toggle == 1:
                self.mesh_view.change_mode('force')
            elif self.toggle == 2:
                self.mesh_view.change_mode('velocity')
            elif self.toggle == 3:
                self.mesh_view.change_mode('vel_dir')
            else:
                self.mesh_view.change_mode('stress')
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
        if self.camera2 is not None:
            self.camera2.remove()
            self.camera2 = None
        if self.lidar is not None:
            self.lidar.remove()
            self.lidar = None
        if self.us0 is not None:
            self.us0.remove()
            self.us0 = None
        if self.us1 is not None:
            self.us1.remove()
            self.us1 = None
        if self.us2 is not None:
            self.us2.remove()
            self.us2 = None
        if self.us3 is not None:
            self.us3.remove()
            self.us3 = None
        if self.us4 is not None:
            self.us4.remove()
            self.us4 = None
        if self.us5 is not None:
            self.us5.remove()
            self.us5 = None
        if self.radar is not None:
            self.radar.remove()
            self.radar = None
        if self.imu1 is not None:
            self.imu1.remove()
            self.imu1 = None
        if self.imu2 is not None:
            self.imu2.remove()
            self.imu2 = None
        if self.mesh is not None:
            self.mesh.remove()
            self.mesh = None

        # Set up the chosen demonstration.
        if self.demo == 'camera':
            self.camera = Camera('camera1', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, resolution=(1820, 980), near_far_planes=(0.01, 1000),
                is_streaming=True)

        elif demo == 'lidar':
            self.lidar = Lidar('lidar', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, is_visualised=False, vertical_resolution=128, frequency=40,
                is_streaming=True)

        elif demo == 'ultrasonic':
            self.us0 = Ultrasonic('us_FL', self.bng, self.main_vehicle, requested_update_time=0.05, is_visualised=False, pos=(10.0, -10.0, 0.5), dir=(1.0, -1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us1 = Ultrasonic('us_FR', self.bng, self.main_vehicle, requested_update_time=0.05, is_visualised=False, pos=(-10.0, -10.0, 0.5), dir=(-1.0, -1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us2 = Ultrasonic('us_BL', self.bng, self.main_vehicle, requested_update_time=0.05, is_visualised=False, pos=(10.0, 10.0, 0.5), dir=(1.0, 1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us3 = Ultrasonic('us_BR', self.bng, self.main_vehicle, requested_update_time=0.05, is_visualised=False, pos=(-10.0, 10.0, 0.5), dir=(-1.0, 1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us4 = Ultrasonic('us_ML', self.bng, self.main_vehicle, requested_update_time=0.05, is_visualised=False, pos=(10.0, 0.0, 0.5), dir=(1.0, 0.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us5 = Ultrasonic('us_MR', self.bng, self.main_vehicle, requested_update_time=0.05, is_visualised=False, pos=(-10.0, 0.0, 0.5), dir=(-1.0, 0.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_view = US_View(self.bng, self.main_vehicle, self.us0, self.us1, self.us2, self.us3, self.us4, self.us5)

        elif demo == 'radar':
            self.radar = Radar('radar1', self.bng, self.main_vehicle, requested_update_time=0.05, pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1), resolution=(self.radar_res[0], self.radar_res[1]),
                field_of_view_y=self.radar_fov, near_far_planes=(0.1, self.radar_range_max), range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                range_min_cutoff=0.5, range_direct_max_cutoff=self.radar_range_max, range_bins=self.radar_bins[0], azimuth_bins=self.radar_bins[1], vel_bins=self.radar_bins[2],
                is_streaming=True)

        elif demo == 'imu':
            self.imu1 = AdvancedIMU('imu1', self.bng, self.main_vehicle, pos=(0.0, 0.0, 0.5), dir=(0, -1, 0), up=(1, 0, 0), gfx_update_time=0.0001, physics_update_time=0.0001,
                is_using_gravity=True, is_visualised=False, is_snapping_desired=True, is_force_inside_triangle=True, accel_window_width=200)
            self.time_series1 = Time_Series(size=2000, x_min=100.0, x_max=800, y_min=100.0, y_max=350.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-25.0, data_max=25.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series2 = Time_Series(size=2000, x_min=100.0, x_max=800, y_min=400.0, y_max=650.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-25.0, data_max=25.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series3 = Time_Series(size=2000, x_min=100.0, x_max=800, y_min=700.0, y_max=950.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-25.0, data_max=25.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series4 = Time_Series(size=2000, x_min=1000.0, x_max=1700, y_min=100.0, y_max=350.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-5.0, data_max=5.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series5 = Time_Series(size=2000, x_min=1000.0, x_max=1700, y_min=400.0, y_max=650.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-5.0, data_max=5.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series6 = Time_Series(size=2000, x_min=1000.0, x_max=1700, y_min=700.0, y_max=950.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-5.0, data_max=5.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)

        elif demo == 'mesh':
            self.mesh = Mesh('mesh', self.bng, self.main_vehicle, gfx_update_time=0.001)
            self.mesh_view = Mesh_View(self.mesh, mass_min=0.0, mass_max=10.0, force_min=0.0, force_max=300.0, vel_min=0.0, vel_max=50.0, stress_min=0.0, stress_max=200.0,
                top_center=vec3(600.0, 840.0), top_scale=vec3(150.0, 150.0), front_center=vec3(600.0, 200.0), front_scale=vec3(150.0, 150.0), right_center=vec3(1340.0, 200.0),
                right_scale=vec3(150.0, 150.0), is_top=True, is_front=True, is_right=True)

        elif demo == 'trajectory':
            self.trajectory_view = Trajectory(memory=10000, x_min=0, x_max=self.width, y_min=0, y_max=self.height, origin_x=self.width * 0.5, origin_y=self.height * 0.5)

        elif demo == 'multi':    # Camera, LiDAR, RADAR, and Ultrasonic together in one view (four viewports).
            self.camera = Camera('camera1', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, resolution=(910, 490), near_far_planes=(0.01, 1000),
                is_render_annotations=False, is_render_depth=False, is_streaming=True)
            self.lidar = Lidar('lidar', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, is_visualised=False, vertical_resolution=64, frequency=40,
                rays_per_second=1500000, is_streaming=True)
            self.radar = Radar('radar1', self.bng, self.main_vehicle, requested_update_time=0.05, pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1), resolution=(self.radar_res[0], self.radar_res[1]),
                field_of_view_y=self.radar_fov, near_far_planes=(0.1, self.radar_range_max), range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                range_min_cutoff=0.5, range_direct_max_cutoff=self.radar_range_max, range_bins=self.radar_bins[0], azimuth_bins=self.radar_bins[1], vel_bins=self.radar_bins[2],
                is_streaming=True)
            self.us0 = Ultrasonic('us_FL', self.bng, self.main_vehicle, requested_update_time=0.1, is_visualised=False, pos=(10.0, -10.0, 0.5), dir=(1.0, -1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us1 = Ultrasonic('us_FR', self.bng, self.main_vehicle, requested_update_time=0.1, is_visualised=False, pos=(-10.0, -10.0, 0.5), dir=(-1.0, -1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us2 = Ultrasonic('us_BL', self.bng, self.main_vehicle, requested_update_time=0.1, is_visualised=False, pos=(10.0, 10.0, 0.5), dir=(1.0, 1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us3 = Ultrasonic('us_BR', self.bng, self.main_vehicle, requested_update_time=0.1, is_visualised=False, pos=(-10.0, 10.0, 0.5), dir=(-1.0, 1.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us4 = Ultrasonic('us_ML', self.bng, self.main_vehicle, requested_update_time=0.1, is_visualised=False, pos=(10.0, 0.0, 0.5), dir=(1.0, 0.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us5 = Ultrasonic('us_MR', self.bng, self.main_vehicle, requested_update_time=0.1, is_visualised=False, pos=(-10.0, 0.0, 0.5), dir=(-1.0, 0.0, 0.1), resolution=(50, 50),
                is_snapping_desired=True, is_force_inside_triangle=True, range_roundess=-125.0)
            self.us_view = US_View(self.bng, self.main_vehicle, self.us0, self.us1, self.us2, self.us3, self.us4, self.us5)

        elif demo == '1':
            self.imu1 = AdvancedIMU('imu1', self.bng, self.main_vehicle, pos=(0.0, 0.0, 0.5), dir=(0, -1, 0), up=(1, 0, 0), gfx_update_time=0.0001, physics_update_time=0.0001, is_using_gravity=True, is_visualised=False,
                is_snapping_desired=True, is_force_inside_triangle=True, accel_window_width=200)
            self.time_series1 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=100.0, y_max=350.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-18.0, data_max=18.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series2 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=400.0, y_max=650.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-18.0, data_max=18.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series3 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=700.0, y_max=950.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-18.0, data_max=18.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.mesh = Mesh('mesh', self.bng, self.main_vehicle, gfx_update_time=0.0005)
            self.mesh_view = Mesh_View(self.mesh, mass_min=0.0, mass_max=10.0, force_min=0.0, force_max=250.0, top_center=vec3(1500.0, 785.0), top_scale=vec3(190.0, 190.0),
                front_center=vec3(1500.0, 215.0), front_scale=vec3(190.0, 190.0), is_top=True, is_front=True, is_right=False)
            self.mesh_view.data_mode = 'force'

        elif demo == '2':
            self.imu1 = AdvancedIMU('imu1', self.bng, self.main_vehicle, pos=(0.0, 0.0, 0.5), dir=(0, -1, 0), up=(1, 0, 0), gfx_update_time=0.0001, physics_update_time=0.0001, is_using_gravity=True, is_visualised=False,
                is_snapping_desired=True, is_force_inside_triangle=True, gyro_window_width=50)
            self.time_series1 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=100.0, y_max=350.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-2.5, data_max=2.5,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series2 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=400.0, y_max=650.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-2.5, data_max=2.5,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series3 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=700.0, y_max=950.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-2.5, data_max=2.5,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.trajectory_view = Trajectory(x_min=960, x_max=1920, y_min=540, y_max=1080, origin_x=1800, origin_y=810, zoom=8.0, rot_deg=-40.0)
            self.mesh = Mesh('mesh', self.bng, self.main_vehicle, gfx_update_time=0.0005)
            self.mesh_view = Mesh_View(self.mesh, mass_min=0.0, mass_max=10.0, top_center=vec3(1500.0, 290.0), top_scale=vec3(150.0, 150.0),
                is_top=True, is_front=False, is_right=False)
            self.mesh_view.data_mode = 'vel_dir'

        elif demo == '3':
            self.imu1 = AdvancedIMU('imu1', self.bng, self.main_vehicle, pos=(0.0, 0.0, 0.5), dir=(0, -1, 0), up=(1, 0, 0), gfx_update_time=0.0001, physics_update_time=0.0001, is_using_gravity=True, is_visualised=False,
                is_snapping_desired=True, is_force_inside_triangle=True, accel_window_width=200)
            self.time_series1 = Time_Series(size=1000, x_min=100.0, x_max=800, y_min=100.0, y_max=350.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-2.5, data_max=2.5,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series2 = Time_Series(size=1000, x_min=100.0, x_max=800, y_min=400.0, y_max=650.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-2.5, data_max=2.5,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series3 = Time_Series(size=1000, x_min=100.0, x_max=800, y_min=700.0, y_max=950.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-2.5, data_max=2.5,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.camera = Camera('camera1', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, pos=(-2.5, 0, 0.35), dir=(1, 0, 0),
                resolution=(910, 490), near_far_planes=(0.1, 100), is_render_annotations=False, is_render_depth=False, is_streaming=True)
            self.mesh = Mesh('mesh', self.bng, self.main_vehicle, gfx_update_time=0.0005)
            self.mesh_view = Mesh_View(self.mesh, mass_min=0.0, mass_max=10.0, force_min=0.0, force_max=600.0, front_center=vec3(1525.0, 200.0), front_scale=vec3(170.0, 170.0),
                is_top=False, is_front=True, is_right=False)
            self.mesh_view.data_mode = 'force'

        elif demo == '4':
            self.imu1 = AdvancedIMU('imu1', self.bng, self.main_vehicle, pos=(0.0, 0.0, 0.5), dir=(0, -1, 0), up=(1, 0, 0), gfx_update_time=0.0001, physics_update_time=0.0001, is_using_gravity=True, is_visualised=False,
                is_snapping_desired=True, is_force_inside_triangle=True, accel_window_width=200)
            self.time_series1 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=100.0, y_max=350.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-100.0, data_max=100.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series2 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=400.0, y_max=650.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-100.0, data_max=100.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series3 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=700.0, y_max=950.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-100.0, data_max=100.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.camera = Camera('camera1', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, pos=(-2.5, 0, 0.35), dir=(1, 0, 0),
                resolution=(910, 490), near_far_planes=(0.1, 100), is_render_annotations=False, is_render_depth=False, is_streaming=True)
            self.mesh = Mesh('mesh', self.bng, self.main_vehicle, gfx_update_time=0.0005)
            self.mesh_view = Mesh_View(self.mesh, mass_min=0.0, mass_max=10.0, force_min=0.0, force_max=1000.0, front_center=vec3(1525.0, 200.0), front_scale=vec3(170.0, 170.0),
                is_top=False, is_front=True, is_right=False)
            self.mesh_view.data_mode = 'force'

        elif demo == '5':
            self.imu1 = AdvancedIMU('imu1', self.bng, self.main_vehicle, pos=(0.0, 0.0, 0.5), dir=(0, -1, 0), up=(1, 0, 0), gfx_update_time=0.0001, physics_update_time=0.0001, is_using_gravity=True, is_visualised=False,
                is_snapping_desired=True, is_force_inside_triangle=True, accel_window_width=200)
            self.time_series1 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=100.0, y_max=350.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-18.0, data_max=18.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series2 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=400.0, y_max=650.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-18.0, data_max=18.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series3 = Time_Series(size=10000, x_min=100.0, x_max=800, y_min=700.0, y_max=950.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-18.0, data_max=18.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.camera = Camera('camera1', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, pos=(-1.4, 1.3, 0.1), dir=(1, 0, 0),
                resolution=(480, 480), near_far_planes=(0.1, 100), is_render_annotations=False, is_render_depth=False, is_streaming=True)
            self.camera2 = Camera('camera2', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, pos=(0.6, -2.0, 0.15), dir=(0, 1, 0),
                resolution=(480, 480), near_far_planes=(0.1, 100), is_render_annotations=False, is_render_depth=False, is_streaming=True)
            self.mesh = Mesh('mesh', self.bng, self.main_vehicle, gfx_update_time=0.0005)
            self.mesh_view = Mesh_View(self.mesh, mass_min=0.0, mass_max=10.0, force_min=0.0, force_max=250.0,
                front_center=vec3(1250.0, 185.0), front_scale=vec3(125.0, 125.0),
                right_center=vec3(1580.0, 185.0), right_scale=vec3(125.0, 125.0),
                is_top=False, is_front=True, is_right=True)
            self.mesh_view.data_mode = 'force'

        elif demo == '6':
            self.imu1 = AdvancedIMU('imu1', self.bng, self.main_vehicle, pos=(0.0, -2.0, 0.5), dir=(0, -1, 0), up=(1, 0, 0), gfx_update_time=0.0001, physics_update_time=0.0001,
                is_using_gravity=True, is_visualised=False, is_snapping_desired=True, is_force_inside_triangle=True, accel_window_width=150)
            self.imu2 = AdvancedIMU('imu2', self.bng, self.main_vehicle, pos=(0.0, 7.0, 4.5), dir=(0, -1, 0), up=(1, 0, 0), gfx_update_time=0.0001, physics_update_time=0.0001,
                is_using_gravity=True, is_visualised=False, is_snapping_desired=True, is_force_inside_triangle=True, accel_window_width=150)
            self.time_series1 = Time_Series(size=1000, x_min=80.0, x_max=670, y_min=730.0, y_max=980.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-300.0, data_max=300.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series2 = Time_Series(size=1000, x_min=720.0, x_max=1310, y_min=730.0, y_max=980.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-300.0, data_max=300.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series3 = Time_Series(size=1000, x_min=1360.0, x_max=1900, y_min=730.0, y_max=980.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-300.0, data_max=300.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series4 = Time_Series(size=1000, x_min=80.0, x_max=670, y_min=430.0, y_max=680.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-300.0, data_max=300.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series5 = Time_Series(size=1000, x_min=720.0, x_max=1310, y_min=430.0, y_max=680.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-300.0, data_max=300.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.time_series6 = Time_Series(size=1000, x_min=1360.0, x_max=1900, y_min=430.0, y_max=680.0, grid_spacing_x=10, grid_spacing_y=4, data_min=-300.0, data_max=300.0,
                axes_overlap_x=10.0, axes_overlap_y=10.0, grid_notch_x=5.0, grid_notch_y=5.0)
            self.camera = Camera('camera1', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, pos=(-3.0, -1.1, 0.4), dir=(1, 0, 0),
                resolution=(370, 370), near_far_planes=(0.1, 100), is_render_annotations=False, is_render_depth=False, is_streaming=True)
            self.camera2 = Camera('camera2', self.bng, self.main_vehicle, requested_update_time=0.05, is_using_shared_memory=True, pos=(0.0, -1.5, 3.0), dir=(0.0, 0.1, -1),
                resolution=(370, 370), near_far_planes=(0.1, 30), is_render_annotations=False, is_render_depth=False, is_streaming=True)

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

        # Fetch the latest position of the main vehicle.
        self.main_vehicle.sensors.poll()
        current_pos = self.main_vehicle.state['pos']
        current_dir = self.main_vehicle.state['dir']

        # Handle any geometric target logic for the chosen scenario.
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
                self.focus = current_pos
                self.pos[0] = self.focus[0] + current_dir[0] * -30
                self.pos[1] = self.focus[1] + current_dir[1] * -30
                self.pos[2] = self.focus[2] + current_dir[2] + 10

        elif self.demo == 'ultrasonic':
            self.us_view.update()

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
            self.time_series1.update(accX)
            self.time_series2.update(accY)
            self.time_series3.update(accZ)
            self.time_series4.update(gyroX)
            self.time_series5.update(gyroY)
            self.time_series6.update(gyroZ)

        elif self.demo == 'mesh':
            self.mesh_view.update(dir=current_dir)

        elif self.demo == 'trajectory':
            self.trajectory_view.update(current_pos)

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
            self.colours[0:self.points_count:3] /= max(1e-12, max_height)
            self.colours[1:self.points_count:3] = 0.25
            self.colours[2:self.points_count:3] = 1.0 - self.colours[0:self.points_count:3]
            glDeleteBuffers(1, self.colour_buf)
            self.colour_buf = np.uint64(glGenBuffers(1))
            glBindBuffer(GL_ARRAY_BUFFER, self.colour_buf)
            glBufferData(GL_ARRAY_BUFFER, self.points_count * 4, self.colours, GL_STATIC_DRAW)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
            if self.follow and self.main_vehicle.state:
                self.focus = current_pos
                self.pos[0] = self.focus[0] + current_dir[0] * -30
                self.pos[1] = self.focus[1] + current_dir[1] * -30
                self.pos[2] = self.focus[2] + current_dir[2] + 10

            # Multi: RADAR update.
            ppi_data = self.radar.stream_ppi()
            self.radar_ppi_size = [self.radar_bins[0], self.radar_bins[1]]
            self.radar_ppi_img = ppi_data

            # Multi: Ultrasonic update.
            self.us_view.update()

        elif self.demo == '1':
            full_imu_data = self.imu1.poll()                                                # update IMU sensor.
            accX, accY, accZ = [], [], []
            for i in range(len(full_imu_data)):
                new_imu_data = full_imu_data[i]
                acc = new_imu_data['accSmooth']
                accX.append(acc[0])
                accY.append(acc[1])
                accZ.append(acc[2])
            self.time_series1.update(accX)
            self.time_series2.update(accY)
            self.time_series3.update(accZ)
            self.mesh_view.update(dir=current_dir)                                          # update Mesh sensor.

            pos = vec3(current_pos[0], current_pos[1], current_pos[2])
            p = vec3(pos[0], pos[1], pos[2])
            d = p.distance(self.scenario1_target)
            if self.scenario1_is_target_hit == True:
                dt = time.time() - self.scenario1_hit_time
                if dt > 0.8:
                    self.mesh_view.pause(True)                                              # pause the mesh after 0.8s.
            elif d < 7.0:
                self.scenario1_is_target_hit = True
                self.scenario1_hit_time = time.time()
                self.vehicles['vehicle_1'].ai.set_mode('disabled')
                self.vehicles['vehicle_1'].control(brake=1.0, parkingbrake=1.0, gear=0, steering=0.0)
                self.time_series1.mark(idx=3000)
                self.time_series2.mark(idx=3000)
                self.time_series3.mark(idx=3000)
            if self.time_series1.is_marker_at_target == True and self.time_series2.is_marker_at_target and self.time_series3.is_marker_at_target:
                self.is_final_display = True

        elif self.demo == '2':
            full_imu_data = self.imu1.poll()                                                # update IMU sensor.
            gyroX, gyroY, gyroZ = [], [], []
            for i in range(len(full_imu_data)):
                new_imu_data = full_imu_data[i]
                gyro = new_imu_data['angVelSmooth']
                gyroX.append(gyro[0])
                gyroY.append(gyro[1])
                gyroZ.append(gyro[2])
            self.time_series1.update(gyroX)
            self.time_series2.update(gyroY)
            self.time_series3.update(gyroZ)
            self.mesh_view.update(dir=current_dir)                                          # update Mesh sensor.
            self.trajectory_view.update(current_pos)                                        # update Trajectory.

            p = vec3(current_pos[0], current_pos[1], current_pos[2])
            d = p.distance(self.scenario2_target)
            if self.scenario2_is_target_hit == True:
                dt = time.time() - self.scenario2_hit_time
                if dt > 0.5 and dt < 1.0:
                    self.vehicles['vehicle_1'].control(steering=-1.0, brake=1.0)            # sc2 hit - step 2.
                elif dt >= 1.0 and dt < 3.3:
                    self.vehicles['vehicle_1'].control(steering=0.0)
                    self.vehicles['vehicle_1'].ai.set_mode('span')
                    self.vehicles['vehicle_3'].ai.set_mode('disabled')                      # stop all the busses.
                    self.vehicles['vehicle_3'].control(steering=0.0, brake=1.0, gear=0)
                    self.vehicles['vehicle_8'].ai.set_mode('disabled')
                    self.vehicles['vehicle_8'].control(steering=0.0, brake=1.0, gear=0)
                    self.vehicles['vehicle_9'].ai.set_mode('disabled')
                    self.vehicles['vehicle_9'].control(steering=0.0, brake=1.0, gear=0)
                    self.vehicles['vehicle_10'].ai.set_mode('disabled')
                    self.vehicles['vehicle_10'].control(steering=0.0, brake=1.0, gear=0)
                    if self.mesh_view.ang_range > math.pi * 0.0625:                         # pause the mesh when the angle variance across the mesh is large enough.
                        self.mesh_view.pause(True)
                elif dt >= 3.3:
                    self.trajectory_view.pause(True)                                        # pause the Trajectory.
                    self.vehicles['vehicle_1'].ai.set_mode('disabled')                      # sc2 hit - step 4.
                    self.vehicles['vehicle_1'].control(steering=0.0, brake=0.3, gear=0)
            elif d < 9.0 and self.scenario2_is_target_hit == False:
                self.scenario2_is_target_hit = True
                self.scenario2_hit_time = time.time()
                self.vehicles['vehicle_1'].ai.set_mode('disabled')                          # sc2 hit - step 1.
                self.vehicles['vehicle_1'].control(steering=1.0, brake=1.0)
                self.time_series1.mark(idx=50)
                self.time_series2.mark(idx=50)
                self.time_series3.mark(idx=50)
            if self.time_series1.is_marker_at_target == True and self.time_series2.is_marker_at_target and self.time_series3.is_marker_at_target:
                self.is_final_display = True

        elif self.demo == '3':
            full_imu_data = self.imu1.poll()                                                    # update IMU sensor.
            gyroX, gyroY, gyroZ = [], [], []
            for i in range(len(full_imu_data)):
                new_imu_data = full_imu_data[i]
                gyro = new_imu_data['angVelSmooth']
                gyroX.append(gyro[0])
                gyroY.append(gyro[1])
                gyroZ.append(gyro[2])
            self.time_series1.update(gyroX)
            self.time_series2.update(gyroY)
            self.time_series3.update(gyroZ)
            self.mesh_view.update(dir=current_dir)                                              # update Mesh sensor.
            if self.is_scenario_3_paused == False:
                cam_width, cam_height = self.camera.resolution[0], self.camera.resolution[1]    # update Camera sensor (unless it has been paused).
                camera_data1 = self.camera.stream_colour(cam_width * cam_height * 4)
                self.camera_color_size = [cam_width, cam_height]
                self.camera_color_img = camera_data1

            pos = vec3(current_pos[0], current_pos[1], current_pos[2])
            p = vec3(pos[0], pos[1], pos[2])
            d = p.distance(self.scenario3_target)
            if d < 4.0:
                if self.mesh_view.force_range > 550.0:
                    self.mesh_view.pause(True)                                                  # pause the mesh when the force variance is large enough.
                self.time_series1.mark(idx=500)
                self.time_series2.mark(idx=500)
                self.time_series3.mark(idx=500)
                self.is_scenario_3_paused = True
            if self.time_series1.is_marker_at_target == True and self.time_series2.is_marker_at_target and self.time_series3.is_marker_at_target:
                self.is_final_display = True

        elif self.demo == '4':
            full_imu_data = self.imu1.poll()                                                    # update IMU sensor.
            accX, accY, accZ = [], [], []
            for i in range(len(full_imu_data)):
                new_imu_data = full_imu_data[i]
                acc = new_imu_data['accSmooth']
                accX.append(acc[0])
                accY.append(acc[1])
                accZ.append(acc[2])
            self.time_series1.update(accX)
            self.time_series2.update(accY)
            self.time_series3.update(accZ)
            self.mesh_view.update(dir=current_dir)                                              # update Mesh sensor.
            if self.is_scenario_4_paused == False:
                cam_width, cam_height = self.camera.resolution[0], self.camera.resolution[1]    # update Camera sensor (unless it has been paused).
                camera_data1 = self.camera.stream_colour(cam_width * cam_height * 4)
                self.camera_color_size = [cam_width, cam_height]
                self.camera_color_img = camera_data1

            pos = vec3(current_pos[0], current_pos[1], current_pos[2])
            p = vec3(pos[0], pos[1], pos[2])
            d = p.distance(self.scenario4_target)
            if d < 5.0:
                if self.mesh_view.force_range > 950.0:
                    self.mesh_view.pause(True)
                self.is_scenario_4_paused = True
                self.time_series1.mark(idx=2500)
                self.time_series2.mark(idx=2500)
                self.time_series3.mark(idx=2500)
            if self.time_series1.is_marker_at_target == True and self.time_series2.is_marker_at_target and self.time_series3.is_marker_at_target:
                self.is_final_display = True

        elif self.demo == '5':
            full_imu_data = self.imu1.poll()                                                    # update IMU sensor.
            accX, accY, accZ = [], [], []
            for i in range(len(full_imu_data)):
                new_imu_data = full_imu_data[i]
                acc = new_imu_data['accSmooth']
                accX.append(acc[0])
                accY.append(acc[1])
                accZ.append(acc[2])
            self.time_series1.update(accX)
            self.time_series2.update(accY)
            self.time_series3.update(accZ)
            self.mesh_view.update(dir=current_dir)                                              # update Mesh sensor.
            if self.is_scenario_5_paused == False:
                cam_width, cam_height = self.camera.resolution[0], self.camera.resolution[1]    # update Camera sensors (unless paused).
                cam_size = cam_width * cam_height * 4
                camera_data1 = self.camera.stream_colour(cam_size)
                self.camera_color_size = [cam_width, cam_height]
                self.camera_color_img = camera_data1
                camera_data2 = self.camera2.stream_colour(cam_size)
                self.camera2_color_size = [cam_width, cam_height]
                self.camera2_color_img = camera_data2

            pos = vec3(current_pos[0], current_pos[1], current_pos[2])
            p = vec3(pos[0], pos[1], pos[2])
            d = p.distance(self.scenario5_target)
            if self.scenario5_is_target_hit == True:
                dt = time.time() - self.scenario5_hit_time
                if dt > 0.4:
                    self.mesh_view.pause(True)                                                  # pause the mesh.
                    self.is_scenario_5_paused = True
                    self.vehicles['vehicle_1'].ai.set_mode('span')
                if dt > 3.0:
                    self.vehicles['vehicle_6'].ai.set_mode('disabled')                          # force other vehicle to brake.
                    self.vehicles['vehicle_6'].control(steering=0.0, brake=1.0, gear=0)
            elif d < 8.0:
                self.scenario5_is_target_hit = True
                self.scenario5_hit_time = time.time()
                self.vehicles['vehicle_1'].ai.set_mode('disabled')
                self.vehicles['vehicle_1'].deflate_tire(0)                                      # deflate the FL tyre.
                self.vehicles['vehicle_7'].ai.set_mode('disabled')                              # force other vehicle to brake.
                self.vehicles['vehicle_7'].control(steering=0.0, brake=1.0, gear=0)
                self.time_series1.mark(idx=2500)
                self.time_series2.mark(idx=2500)
                self.time_series3.mark(idx=2500)
            if self.time_series1.is_marker_at_target == True and self.time_series2.is_marker_at_target and self.time_series3.is_marker_at_target:
                self.is_final_display = True

        elif self.demo == '6':
            full_imu_data1 = self.imu1.poll()
            full_imu_data2 = self.imu2.poll()
            acc1X, acc1Y, acc1Z, acc2X, acc2Y, acc2Z, acc2T = [], [], [], [], [], [], []
            sync_time = -1e12
            for i in range(len(full_imu_data1)):
                new_imu_data = full_imu_data1[i]
                acc = new_imu_data['accSmooth']
                acc1X.append(acc[0])
                acc1Y.append(acc[1])
                acc1Z.append(acc[2])
                sync_time = max(sync_time, new_imu_data['time'])
            self.time_series1.update(acc1X)
            self.time_series2.update(acc1Y)
            self.time_series3.update(acc1Z)
            for i in range(len(full_imu_data2)):
                new_imu_data = full_imu_data2[i]
                acc = new_imu_data['accSmooth']
                acc2X.append(acc[0])
                acc2Y.append(acc[1])
                acc2Z.append(acc[2])
                acc2T.append(new_imu_data['time'])
            self.time_series4.update(acc2X, times=acc2T)
            self.time_series5.update(acc2Y, times=acc2T)
            self.time_series6.update(acc2Z, times=acc2T)
            if self.is_scenario_6_paused == False:
                cam_width, cam_height = self.camera.resolution[0], self.camera.resolution[1]    # update Camera sensors (unless paused).
                cam_size = cam_width * cam_height * 4
                camera_data1 = self.camera.stream_colour(cam_size)
                self.camera_color_size = [cam_width, cam_height]
                self.camera_color_img = camera_data1
                camera_data2 = self.camera2.stream_colour(cam_size)
                self.camera2_color_size = [cam_width, cam_height]
                self.camera2_color_img = camera_data2

            pos = vec3(current_pos[0], current_pos[1], current_pos[2])
            p = vec3(pos[0], pos[1], pos[2])
            d = p.distance(self.scenario6_target)
            if self.scenario6_is_target_hit == True:
                dt = time.time() - self.scenario6_hit_time
                if dt > 0.3:
                    self.is_scenario_6_paused = True
            elif d < 3.0:
                self.scenario6_is_target_hit = True
                self.scenario6_hit_time = time.time()
                self.time_series1.mark(idx=100)                                                 # Mark the time series displays so they all stop at the same point.
                self.time_series2.mark(idx=100)
                self.time_series3.mark(idx=100)
                self.time_series4.mark(idx=100, sync_time=sync_time)
                self.time_series5.mark(idx=100, sync_time=sync_time)
                self.time_series6.mark(idx=100, sync_time=sync_time)
            if self.time_series1.is_marker_at_target == True and self.time_series2.is_marker_at_target and self.time_series3.is_marker_at_target:
                self.is_final_display = True

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
            if self.trajectory_view is not None:
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
                traj_lines = self.trajectory_view.display()
                for line in traj_lines:
                    self.draw_line(line)
                if len(traj_lines) > 0:
                    car_x, car_y = traj_lines[-1][2], traj_lines[-1][3]                                             # The car rectangle.
                    glColor3f(0.9, 0.05, 0.05)
                    glRectf(car_x - 10, car_y - 10, car_x + 10, car_y + 10)
                    glColor3f(0.0, 0.0, 0.0)
                    glRectf(car_x - 8, car_y - 8, car_x + 8, car_y + 8)

                glEnable(GL_LINE_SMOOTH)
                glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
                glEnable(GL_BLEND)
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

                # Screen title underline.
                glLineWidth(2.0)
                self.draw_line([40, 1040, 300, 1040])

                # Draw Text.
                glEnable( GL_TEXTURE_2D )
                glBindTexture( GL_TEXTURE_2D, texid )
                glColor3f(0.85, 0.85, 0.70)
                self.draw_text(50, 1060, 'Trajectory')
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
                    glViewport(0, 0, self.width, self.height)
                    self.render_img(50, 50, self.camera_color_img, self.camera_color_size[0], self.camera_color_size[1], 1, 1, 1, 0)

                    # Now deal with the 2D top bar (title etc).
                    glEnable(GL_LINE_SMOOTH)
                    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
                    glEnable(GL_BLEND)
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

                    # Title underline.
                    glColor3f(0.25, 0.25, 0.15)
                    glLineWidth(2.0)
                    self.draw_line([75, 1000, 400, 1000])

                    # Draw Text.
                    glEnable( GL_TEXTURE_2D )
                    glBindTexture( GL_TEXTURE_2D, texid )
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(85, 1020, 'Camera Sensor - Color Image')
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
                    self.draw_line([75, 985, 460, 985])

                    # View-division lines.
                    glLineWidth(3.0)
                    self.draw_line([0, self.annot_height, self.annot_width, self.annot_height])
                    self.draw_line([self.annot_width, 0, self.annot_width, self.height])

                    # Draw the colour rectangles for each class key.
                    glLineWidth(1.0)
                    box_x0, box_x1 = 1440, 1460
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
                    self.draw_text(85, 1000, 'Camera Sensor - Class Annotations')
                    self.draw_text(85, 900, 'Map: ')
                    self.draw_text(1440, 971, 'Color -> Class Map:')
                    glColor3f(0.85, 0.35, 0.70)
                    self.draw_text(85, 900, '     ' + self.map_name)
                    ctr = 0
                    for k in self.annot_map.keys():
                        y_pos = 895 - (ctr * 26)
                        ctr = ctr + 1
                        self.draw_text(1490, y_pos, k)
                    glDisable( GL_TEXTURE_2D )

            elif self.toggle == 2:                                                                                                              # depth image only.
                if len(self.camera_depth_size) > 0:
                    glViewport(0, 0, self.width, self.height)
                    self.render_img(50, 50, self.camera_depth_img, self.camera_depth_size[0], self.camera_depth_size[1], 1, 1, 1, 2)

                    # Now deal with the 2D top bar (title etc).
                    glEnable(GL_LINE_SMOOTH)
                    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
                    glEnable(GL_BLEND)
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

                    # Title underline.
                    glColor3f(0.25, 0.25, 0.15)
                    glLineWidth(2.0)
                    self.draw_line([75, 1000, 400, 1000])

                    # Draw Text.
                    glEnable(GL_TEXTURE_2D)
                    glBindTexture(GL_TEXTURE_2D, texid)
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(85, 1020, 'Camera Sensor - Depth Image')
                    glDisable(GL_TEXTURE_2D)

            else:                                                                                                                               # all images.
                if len(self.camera_color_size) > 0:
                    glViewport(0, 0, self.half_width, self.half_height)
                    self.render_img(50, 50, self.camera_color_img, self.camera_color_size[0], self.camera_color_size[1], 1, 1, 1, 0)
                if len(self.camera_annot_size) > 0:
                    glViewport(self.half_width, 0, self.half_width, self.half_height)
                    self.render_img(50, 50, self.camera_annot_img, self.camera_annot_size[0], self.camera_annot_size[1], 1, 1, 1, 0)
                if len(self.camera_depth_size) > 0:
                    glViewport(0, self.half_height, self.half_width, self.half_height)
                    self.render_img(50, 50, self.camera_depth_img, self.camera_depth_size[0], self.camera_depth_size[1], 1, 1, 1, 2)

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
                    self.draw_line([1000, 1000, 1185, 1000])

                    # Draw Text.
                    glEnable( GL_TEXTURE_2D )
                    glBindTexture( GL_TEXTURE_2D, texid )
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(410, 565, ' Depth Camera')
                    self.draw_text(1310, 25, '  Semantic Annotations')
                    self.draw_text(420, 25, 'Color Camera')
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(1020, 1020, 'Camera Sensor')
                    glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == 'lidar':
            glEnable(GL_LINE_SMOOTH)
            glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            if self.points_count > 0:
                glViewport(0, 0, self.width, self.height)
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

            # Title underline.
            glColor3f(0.25, 0.25, 0.15)
            glLineWidth(2.0)
            self.draw_line([75, 1000, 375, 1000])

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(85, 1020, 'LiDAR Sensor: Point Cloud')
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

            # Render each of the display bars.
            rects = self.us_view.display()
            glColor3f(0.1, 0.1, 0.1)
            for r in rects['grey']:
                glRectf(r[0], r[1], r[2], r[3])
            glColor3f(1.0, 1.0, 1.0)
            for r in rects['white']:
                glRectf(r[0], r[1], r[2], r[3])
            glColor3f(1.0, 1.0, 0.0)
            for r in rects['yellow']:
                glRectf(r[0], r[1], r[2], r[3])
            glColor3f(1.0, 0.0, 0.0)
            for r in rects['red']:
                glRectf(r[0], r[1], r[2], r[3])

            # Title underline.
            glColor3f(0.25, 0.25, 0.15)
            glLineWidth(2.0)
            self.draw_line([55, 1040, 320, 1040])

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(85, 1045, 'Ultrasonic Sensor')
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
            if self.toggle == 0:                                                                                # Range-Doppler-Azimuth.
                if len(self.radar_ppi_size) > 0:
                    glViewport(0, 0, self.width, self.height)

                    # Render the colorbar.
                    self.render_img(69, 49, self.rgb_colorbar, self.rgb_colorbar_size[0], self.rgb_colorbar_size[1], 1, 1, 1, 1)

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

                    # Title underline.
                    glColor3f(0.25, 0.25, 0.15)
                    glLineWidth(2.0)
                    self.draw_line([25, 1020, 475, 1020])

                    # Draw Text.
                    glEnable( GL_TEXTURE_2D )
                    glBindTexture( GL_TEXTURE_2D, texid )
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(35, 1040, 'RADAR Sensor:  Range-Azimuth-Doppler')
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
                    self.draw_text(120, 60, '0 m/s')
                    self.draw_text(120, 257, '25 m/s')
                    self.draw_text(120, 455, '50 m/s')
                    glColor3f(0.5, 0.5, 0.5)
                    self.draw_text(44, 490, 'Velocity')
                    glDisable( GL_TEXTURE_2D )

            else:
                if len(self.radar_rvv_size) > 0:
                    glViewport(0, 0, self.width * 2, self.height * 2)
                    self.render_img(175, 22.5, self.radar_rvv_img, self.radar_rvv_size[0], self.radar_rvv_size[1], 1, 1, 1, 0)

                    # Render the colorbar.
                    glViewport(0, 0, self.width, self.height)
                    self.render_img(69, 49, self.rgb_colorbar, self.rgb_colorbar_size[0], self.rgb_colorbar_size[1], 1, 1, 1, 1)

                    glEnable(GL_LINE_SMOOTH)
                    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
                    glEnable(GL_BLEND)
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

                    # Draw the PPI scope frame.
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

                    # Title underline.
                    glColor3f(0.25, 0.25, 0.15)
                    glLineWidth(2.0)
                    self.draw_line([25, 1020, 290, 1020])

                    # Draw Text.
                    glEnable( GL_TEXTURE_2D )
                    glBindTexture( GL_TEXTURE_2D, texid )
                    glColor3f(0.85, 0.85, 0.70)
                    self.draw_text(35, 1040, 'RADAR: Range - Doppler')
                    glColor3f(0.4, 0.4, 0.4)
                    txt = ['0 m', '10 m', '20 m', '30 m', '40 m', '50 m', '60 m', '70 m', '80 m', '90 m', '100 m']
                    txt2 = ['-50 m/s', '-40 m/s', '-30 m/s', '-20 m/s', '-10 m/s', '0 m/s', '10 m/s', '20 m/s', '30 m/s', '40 m/s', '50 m/s']
                    for i in range(11):
                        dv = i * div
                        y = 52 + dv
                        self.draw_text(1320, y, txt2[i])                        # vertical text.
                        x = 332 + dv
                        self.draw_text(x, 22, txt[i])                           # horizontal text.
                    self.draw_text(120, 60, '0 m/s')
                    self.draw_text(120, 257, '25 m/s')
                    self.draw_text(120, 455, '50 m/s')
                    glColor3f(0.5, 0.5, 0.5)
                    self.draw_text(44, 490, 'Velocity')
                    glColor3f(0.75, 0.75, 0.75)
                    self.draw_text(1420, 525, 'Velocity')
                    self.draw_text(796, 75, 'Range')
                    glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == 'imu':
            # Get display data.
            disp1, disp2, disp3 = self.time_series1.display(), self.time_series2.display(), self.time_series3.display()
            disp4, disp5, disp6 = self.time_series4.display(), self.time_series5.display(), self.time_series6.display()

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
            grid1, grid2, grid3, grid4, grid5, grid6 = disp1['grid'], disp2['grid'], disp3['grid'], disp4['grid'], disp5['grid'], disp6['grid']
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
            self.draw_line_strip(disp1['data'])
            self.draw_line_strip(disp2['data'])
            self.draw_line_strip(disp3['data'])
            self.draw_line_strip(disp4['data'])
            self.draw_line_strip(disp5['data'])
            self.draw_line_strip(disp6['data'])

            # Draw axes.
            glColor3f(1.0, 1.0, 1.0)
            glLineWidth(3.0)
            ax1, ax2, ax3, ax4, ax5, ax6 = disp1['axes'], disp2['axes'], disp3['axes'], disp4['axes'], disp5['axes'], disp6['axes']
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
            self.draw_text(300, 985, 'IMU - TRI-AXIAL ACCELERATION')
            self.draw_text(1200, 985, ' IMU - TRI-AXIAL GYROSCOPIC')
            self.draw_text(1300, 45, 'time (seconds)')
            self.draw_text(400, 45, 'time (seconds)')
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(790, 80, '0s')
            self.draw_text(690, 80, '-0.4s')
            self.draw_text(540, 80, '-0.8s')
            self.draw_text(390, 80, '-1.2s')
            self.draw_text(240, 80, '-1.6s')
            self.draw_text(90, 80, '-2.0s')
            self.draw_text(44, 231, '0')
            self.draw_text(44, 531, '0')
            self.draw_text(44, 831, '0')
            self.draw_text(20, 107, '-25.0')
            self.draw_text(30, 357, '25.0')
            self.draw_text(20, 408, '-25.0')
            self.draw_text(30, 657, '25.0')
            self.draw_text(20, 708, '-25.0')
            self.draw_text(30, 957, '25.0')
            self.draw_text(1690, 80, '0s')
            self.draw_text(1590, 80, '-0.4s')
            self.draw_text(1440, 80, '-0.8s')
            self.draw_text(1290, 80, '-1.2s')
            self.draw_text(1140, 80, '-1.6s')
            self.draw_text(990, 80, '-2s')
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
            mesh_data = self.mesh_view.display()
            top, front, right, colors = mesh_data['top'], mesh_data['front'], mesh_data['right'], mesh_data['colors']

            glViewport(0, 0, self.width, self.height)

            # Render the colorbar.
            self.render_img(1780, 50, self.rgb_colorbar, self.rgb_colorbar_size[0], self.rgb_colorbar_size[1], 1, 1, 1, 1)

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

            # Draw beams.
            glLineWidth(1.0)
            num_beams = len(top['beams'])
            for i in range(num_beams):
                color = colors[i]
                glColor3f(color[0], color[1], color[2])
                line = top['beams'][i]
                self.draw_line([line[0], line[1], line[2], line[3]])
                line = front['beams'][i]
                self.draw_line([line[0], line[1], line[2], line[3]])
                line = right['beams'][i]
                self.draw_line([line[0], line[1], line[2], line[3]])

            # Draw nodes.
            glColor3f(0.75, 0.75, 0.60)
            num_nodes = len(top['nodes'])
            for i in range(num_nodes):
                node = top['nodes'][i]
                glRectf(node[0] - 2, node[1] - 2, node[0] + 2, node[1] + 2)
                node = front['nodes'][i]
                glRectf(node[0] - 2, node[1] - 2, node[0] + 2, node[1] + 2)
                node = right['nodes'][i]
                glRectf(node[0] - 2, node[1] - 2, node[0] + 2, node[1] + 2)

            # View-division lines.
            glColor3f(0.25, 0.25, 0.15)
            glLineWidth(3.0)
            self.draw_line([0, self.half_height, self.width, self.half_height])
            self.draw_line([self.half_width, 0, self.half_width, self.height])

            # Screen title underline.
            glLineWidth(2.0)
            self.draw_line([1020, 928, 1400, 930])

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.85, 0.85, 0.70)
            if self.toggle == 0:
                self.draw_text(1050, 950, 'Vehicle Mass Distribution')
                self.draw_text(1822, 51, '0 kg')
                self.draw_text(1822, 251, '5 kg')
                self.draw_text(1822, 451, '10 kg')

            elif self.toggle == 1:
                self.draw_text(1050, 950, 'Vehicle Force Distribution')
                self.draw_text(1822, 51, '0 N')
                self.draw_text(1822, 251, '150 N')
                self.draw_text(1822, 451, '300 N')

            elif self.toggle == 2:
                self.draw_text(1050, 950, 'Velocity (Size) Distribution')
                self.draw_text(1822, 51, '0 m/s')
                self.draw_text(1822, 251, '25 m/s')
                self.draw_text(1822, 451, '50 m/s')

            elif self.toggle == 3:
                self.draw_text(1050, 950, 'Vehicle Velocity (Direction)')
                self.draw_text(1822, 51, 'Fwds')
                self.draw_text(1822, 251, 'Lat')
                self.draw_text(1822, 451, 'Bwds')

            else:
                self.draw_text(1050, 950, 'Beam Stress Distribution')
                self.draw_text(1822, 51, '-200 Nm-2')
                self.draw_text(1822, 251, '0 Nm-2')
                self.draw_text(1822, 451, '200 Nm-2')

            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(442, 570, ' Top')
            self.draw_text(422, 30, '  Right')
            self.draw_text(1320, 30, '    Front')
            glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == 'multi':

            if len(self.camera_color_size) > 0:                                                                                         # camera - color image.
                glViewport(0, self.half_height, self.half_width * 2, self.half_height * 2)	# TL
                self.render_img(25, 25, self.camera_color_img, self.camera_color_size[0], self.camera_color_size[1], 1, 1, 1, 0)

            glViewport(self.half_width, 0, self.half_width, self.half_height)
            self.render_img(153, 150, self.car_img, self.car_img_size[0], self.car_img_size[1], 1, 1, 1, 1)                             # The Ultrasonic .png image.

            if len(self.radar_ppi_size) > 0:
                glViewport(self.half_width, self.half_height, self.half_width, self.half_height)
                # Render the colorbar.
                self.render_img(69, 49, self.rgb_colorbar, self.rgb_colorbar_size[0], self.rgb_colorbar_size[1], 1, 1, 1, 1)
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

            # Ultrasonic.
            glViewport(self.half_width, 0, self.half_width, self.half_height)
            rects = self.us_view.display()
            glColor3f(0.1, 0.1, 0.1)
            for r in rects['grey']:
                glRectf(r[0], r[1], r[2], r[3])
            glColor3f(1.0, 1.0, 1.0)
            for r in rects['white']:
                glRectf(r[0], r[1], r[2], r[3])
            glColor3f(1.0, 1.0, 0.0)
            for r in rects['yellow']:
                glRectf(r[0], r[1], r[2], r[3])
            glColor3f(1.0, 0.0, 0.0)
            for r in rects['red']:
                glRectf(r[0], r[1], r[2], r[3])

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
            self.draw_text(450, 565, ' Camera')
            self.draw_text(450, 30, '  LiDAR')
            self.draw_text(1335, 30, '    Ultrasonic')
            self.draw_text(1350, 565, '    RADAR')
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
            self.draw_text(120, 60, '0 m/s')
            self.draw_text(120, 257, '25 m/s')
            self.draw_text(120, 455, '50 m/s')
            glColor3f(0.5, 0.5, 0.5)
            self.draw_text(44, 490, 'Velocity')
            glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == '1':
            glViewport(0, 0, self.width, self.height)

            # Render the colorbar.
            self.render_img(1800, 50, self.rgb_colorbar, self.rgb_colorbar_size[0], self.rgb_colorbar_size[1], 1, 1, 1, 1)

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

            # Accelerometer (IMU) render.
            disp1, disp2, disp3 = self.time_series1.display(), self.time_series2.display(), self.time_series3.display()

            # Draw grid.
            grid1, grid2, grid3 = disp1['grid'], disp2['grid'], disp3['grid']
            glColor3f(0.1, 0.1, 0.1)
            glLineWidth(1.0)
            for i in grid1['thin'] + grid2['thin'] + grid3['thin']:
                self.draw_line(i)
            glColor3f(0.2, 0.2, 0.2)
            glLineWidth(2.0)
            for i in grid1['thick'] + grid2['thick'] + grid3['thick']:
                self.draw_line(i)

            # Draw data polylines.
            glColor3f(1.0, 0.0, 0.0)
            self.draw_line_strip(disp1['data'])
            self.draw_line_strip(disp2['data'])
            self.draw_line_strip(disp3['data'])

            # Draw axes.
            glColor3f(1.0, 1.0, 1.0)
            glLineWidth(3.0)
            ax1, ax2, ax3 = disp1['axes'], disp2['axes'], disp3['axes']
            self.draw_line(ax1[0])
            self.draw_line(ax1[1])
            self.draw_line(ax2[0])
            self.draw_line(ax2[1])
            self.draw_line(ax3[0])
            self.draw_line(ax3[1])

            # Mesh render.
            mesh_data = self.mesh_view.display()
            top, front, colors = mesh_data['top'], mesh_data['front'], mesh_data['colors']

            # Draw beams.
            glLineWidth(1.0)
            num_beams = len(top['beams'])
            for i in range(num_beams):
                color = colors[i]
                glColor3f(color[0], color[1], color[2])
                line = top['beams'][i]
                self.draw_line([line[0], line[1], line[2], line[3]])
                line = front['beams'][i]
                self.draw_line([line[0], line[1], line[2], line[3]])

            # Draw nodes.
            glColor3f(0.75, 0.75, 0.60)
            num_nodes = len(top['nodes'])
            for i in range(num_nodes):
                node = top['nodes'][i]
                glRectf(node[0] - 2, node[1] - 2, node[0] + 2, node[1] + 2)
                node = front['nodes'][i]
                glRectf(node[0] - 2, node[1] - 2, node[0] + 2, node[1] + 2)

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.65, 0.65, 0.3)
            self.draw_text(820, 233, 'X')
            self.draw_text(820, 534, 'Y')
            self.draw_text(820, 834, 'Z')
            self.draw_text(350, 985, 'IMU - Acceleration')
            self.draw_text(1160, 985, 'Vehicle Force Distribution')
            self.draw_text(373, 45, 'time (seconds)')
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(794, 80, '0s')
            self.draw_text(646, 80, '-0.5s')
            self.draw_text(506, 80, '-1s')
            self.draw_text(366, 80, '-1.5s')
            self.draw_text(225, 80, '-2s')
            self.draw_text(85, 80, '-2.5s')
            self.draw_text(44, 231, '0')
            self.draw_text(44, 531, '0')
            self.draw_text(44, 831, '0')
            self.draw_text(20, 107, '-18.0')
            self.draw_text(30, 357, '18.0')
            self.draw_text(20, 408, '-18.0')
            self.draw_text(30, 657, '18.0')
            self.draw_text(20, 708, '-18.0')
            self.draw_text(30, 957, '18.0')
            self.draw_text(1840, 456, '300 N')
            self.draw_text(1840, 257, '150 N')
            self.draw_text(1840, 60, '0 N')
            glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        if self.demo == '2':
            glViewport(0, 0, self.width, self.height)

            # Render the colorbar.
            self.render_img(1770, 45, self.rgb_colorbar, self.rgb_colorbar_size[0], self.rgb_colorbar_size[1], 1, 1, 1, 1)

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

            # Gyroscopic (IMU) render.
            disp1, disp2, disp3 = self.time_series1.display(), self.time_series2.display(), self.time_series3.display()

            # Draw grid.
            grid1, grid2, grid3 = disp1['grid'], disp2['grid'], disp3['grid']
            glColor3f(0.1, 0.1, 0.1)
            glLineWidth(1.0)
            for i in grid1['thin'] + grid2['thin'] + grid3['thin']:
                self.draw_line(i)
            glColor3f(0.2, 0.2, 0.2)
            glLineWidth(2.0)
            for i in grid1['thick'] + grid2['thick'] + grid3['thick']:
                self.draw_line(i)

            # Draw data polylines.
            glColor3f(1.0, 0.0, 0.0)
            self.draw_line_strip(disp1['data'])
            self.draw_line_strip(disp2['data'])
            self.draw_line_strip(disp3['data'])

            # Draw axes.
            glColor3f(1.0, 1.0, 1.0)
            glLineWidth(3.0)
            ax1, ax2, ax3 = disp1['axes'], disp2['axes'], disp3['axes']
            self.draw_line(ax1[0])
            self.draw_line(ax1[1])
            self.draw_line(ax2[0])
            self.draw_line(ax2[1])
            self.draw_line(ax3[0])
            self.draw_line(ax3[1])

            # Mesh render.
            mesh_data = self.mesh_view.display()
            top, colors = mesh_data['top'], mesh_data['colors']

            # Draw beams.
            glLineWidth(1.0)
            num_beams = len(top['beams'])
            for i in range(num_beams):
                color = colors[i]
                glColor3f(color[0], color[1], color[2])
                line = top['beams'][i]
                self.draw_line([line[0], line[1], line[2], line[3]])

            # Draw nodes.
            glColor3f(0.75, 0.75, 0.60)
            num_nodes = len(top['nodes'])
            for i in range(num_nodes):
                node = top['nodes'][i]
                glRectf(node[0] - 2, node[1] - 2, node[0] + 2, node[1] + 2)

            # Plot the trajectory.
            glColor3f(0.7, 0.35, 0.7)
            glLineWidth(2.0)
            traj_lines = self.trajectory_view.display()
            for line in traj_lines:
                self.draw_line(line)
            if len(traj_lines) > 0:
                car_x, car_y = traj_lines[-1][2], traj_lines[-1][3]                                             # The car rectangle.
                glColor3f(0.9, 0.05, 0.05)
                glRectf(car_x - 20, car_y - 10, car_x + 20, car_y + 10)
                glColor3f(0.0, 0.0, 0.0)
                glRectf(car_x - 18, car_y - 8, car_x + 18, car_y + 8)

            # View-division lines.
            glViewport(0, 0, self.width, self.height)
            glColor3f(0.25, 0.25, 0.15)
            glLineWidth(3.0)
            self.draw_line([self.half_width, self.half_height, self.width, self.half_height])
            self.draw_line([self.half_width, 0, self.half_width, self.height])

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.65, 0.65, 0.3)
            self.draw_text(820, 233, 'Roll [X]')
            self.draw_text(820, 534, 'Pitch [Y]')
            self.draw_text(820, 834, 'Yaw [Z]')
            self.draw_text(358, 995, 'IMU - Gyroscopic')
            self.draw_text(1330, 500, 'Vehicle Velocity Direction')
            self.draw_text(1400, 995, 'Trajectory')
            self.draw_text(373, 47, 'time (seconds)')
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(795, 80, '0s')
            self.draw_text(645, 80, '-2s')
            self.draw_text(505, 80, '-4s')
            self.draw_text(364, 80, '-6s')
            self.draw_text(223, 80, '-8s')
            self.draw_text(84, 80, '-10s')
            self.draw_text(44, 231, '0')
            self.draw_text(44, 531, '0')
            self.draw_text(44, 831, '0')
            self.draw_text(20, 107, '-2.5')
            self.draw_text(30, 357, '2.5')
            self.draw_text(20, 408, '-2.5')
            self.draw_text(30, 657, '2.5')
            self.draw_text(20, 708, '-2.5')
            self.draw_text(30, 957, '2.5')
            self.draw_text(1810, 451, '22.5 deg')
            self.draw_text(1810, 252, '11.25 deg')
            self.draw_text(1810, 55, 'Fwd')
            glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == '3':
            glViewport(0, 0, self.width, self.height)

            # Render the Camera colour image.
            if len(self.camera_color_size) > 0:
                self.render_img(985, 565, self.camera_color_img, self.camera_color_size[0], self.camera_color_size[1], 1, 1, 1, 0)

            # Render the colorbar.
            self.render_img(1770, 45, self.rgb_colorbar, self.rgb_colorbar_size[0], self.rgb_colorbar_size[1], 1, 1, 1, 1)

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

            # Gyroscopic (IMU) render.
            disp1, disp2, disp3 = self.time_series1.display(), self.time_series2.display(), self.time_series3.display()

            # Draw grid.
            grid1, grid2, grid3 = disp1['grid'], disp2['grid'], disp3['grid']
            glColor3f(0.1, 0.1, 0.1)
            glLineWidth(1.0)
            for i in grid1['thin'] + grid2['thin'] + grid3['thin']:
                self.draw_line(i)
            glColor3f(0.2, 0.2, 0.2)
            glLineWidth(2.0)
            for i in grid1['thick'] + grid2['thick'] + grid3['thick']:
                self.draw_line(i)

            # Draw data polylines.
            glColor3f(1.0, 0.0, 0.0)
            self.draw_line_strip(disp1['data'])
            self.draw_line_strip(disp2['data'])
            self.draw_line_strip(disp3['data'])

            # Draw axes.
            glColor3f(1.0, 1.0, 1.0)
            glLineWidth(3.0)
            ax1, ax2, ax3 = disp1['axes'], disp2['axes'], disp3['axes']
            self.draw_line(ax1[0])
            self.draw_line(ax1[1])
            self.draw_line(ax2[0])
            self.draw_line(ax2[1])
            self.draw_line(ax3[0])
            self.draw_line(ax3[1])

            # Mesh render.
            mesh_data = self.mesh_view.display()
            front, colors = mesh_data['front'], mesh_data['colors']

            # Draw beams.
            glLineWidth(1.0)
            num_beams = len(front['beams'])
            for i in range(num_beams):
                color = colors[i]
                glColor3f(color[0], color[1], color[2])
                line = front['beams'][i]
                self.draw_line([line[0], line[1], line[2], line[3]])

            # Draw nodes.
            glColor3f(0.75, 0.75, 0.60)
            num_nodes = len(front['nodes'])
            for i in range(num_nodes):
                node = front['nodes'][i]
                glRectf(node[0] - 2, node[1] - 2, node[0] + 2, node[1] + 2)

            # View-division lines.
            glViewport(0, 0, self.width, self.height)
            glColor3f(0.25, 0.25, 0.15)
            glLineWidth(3.0)
            self.draw_line([self.half_width, self.half_height, self.width, self.half_height])
            self.draw_line([self.half_width, 0, self.half_width, self.height])

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.65, 0.65, 0.3)
            self.draw_text(820, 233, 'Roll [X]')
            self.draw_text(820, 534, 'Pitch [Y]')
            self.draw_text(820, 834, 'Yaw [Z]')
            self.draw_text(358, 995, 'IMU - Gyroscopic')
            self.draw_text(1330, 500, 'Force Distribution')
            self.draw_text(373, 47, 'time (seconds)')
            glColor3f(0.1, 0.1, 0.1)
            self.draw_text(1400, 1040, 'Side Camera')
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(795, 80, '0s')
            self.draw_text(645, 80, '-0.1s')
            self.draw_text(505, 80, '-0.2s')
            self.draw_text(364, 80, '-0.3s')
            self.draw_text(223, 80, '-0.4s')
            self.draw_text(84, 80, '-0.5s')
            self.draw_text(44, 231, '0')
            self.draw_text(44, 531, '0')
            self.draw_text(44, 831, '0')
            self.draw_text(20, 107, '-2.5')
            self.draw_text(30, 357, '2.5')
            self.draw_text(20, 408, '-2.5')
            self.draw_text(30, 657, '2.5')
            self.draw_text(20, 708, '-2.5')
            self.draw_text(30, 957, '2.5')
            self.draw_text(1810, 451, '600 N')
            self.draw_text(1810, 252, '300 N')
            self.draw_text(1810, 55, '0 N')
            glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == '4':
            glViewport(0, 0, self.width, self.height)

            # Render the Camera colour image.
            if len(self.camera_color_size) > 0:
                self.render_img(985, 565, self.camera_color_img, self.camera_color_size[0], self.camera_color_size[1], 1, 1, 1, 0)

            # Render the colorbar.
            self.render_img(1770, 45, self.rgb_colorbar, self.rgb_colorbar_size[0], self.rgb_colorbar_size[1], 1, 1, 1, 1)

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

            # Gyroscopic (IMU) render.
            disp1, disp2, disp3 = self.time_series1.display(), self.time_series2.display(), self.time_series3.display()

            # Draw grid.
            grid1, grid2, grid3 = disp1['grid'], disp2['grid'], disp3['grid']
            glColor3f(0.1, 0.1, 0.1)
            glLineWidth(1.0)
            for i in grid1['thin'] + grid2['thin'] + grid3['thin']:
                self.draw_line(i)
            glColor3f(0.2, 0.2, 0.2)
            glLineWidth(2.0)
            for i in grid1['thick'] + grid2['thick'] + grid3['thick']:
                self.draw_line(i)

            # Draw data polylines.
            glColor3f(1.0, 0.0, 0.0)
            self.draw_line_strip(disp1['data'])
            self.draw_line_strip(disp2['data'])
            self.draw_line_strip(disp3['data'])

            # Draw axes.
            glColor3f(1.0, 1.0, 1.0)
            glLineWidth(3.0)
            ax1, ax2, ax3 = disp1['axes'], disp2['axes'], disp3['axes']
            self.draw_line(ax1[0])
            self.draw_line(ax1[1])
            self.draw_line(ax2[0])
            self.draw_line(ax2[1])
            self.draw_line(ax3[0])
            self.draw_line(ax3[1])

            # Mesh render.
            mesh_data = self.mesh_view.display()
            front, colors = mesh_data['front'], mesh_data['colors']

            # Draw beams.
            glLineWidth(1.0)
            num_beams = len(front['beams'])
            for i in range(num_beams):
                color = colors[i]
                glColor3f(color[0], color[1], color[2])
                line = front['beams'][i]
                self.draw_line([line[0], line[1], line[2], line[3]])

            # Draw nodes.
            glColor3f(0.75, 0.75, 0.60)
            num_nodes = len(front['nodes'])
            for i in range(num_nodes):
                node = front['nodes'][i]
                glRectf(node[0] - 2, node[1] - 2, node[0] + 2, node[1] + 2)

            # View-division lines.
            glViewport(0, 0, self.width, self.height)
            glColor3f(0.25, 0.25, 0.15)
            glLineWidth(3.0)
            self.draw_line([self.half_width, self.half_height, self.width, self.half_height])
            self.draw_line([self.half_width, 0, self.half_width, self.height])

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.65, 0.65, 0.3)
            self.draw_text(820, 233, 'X')
            self.draw_text(820, 534, 'Y')
            self.draw_text(820, 834, 'Z')
            self.draw_text(358, 995, 'IMU - Acceleration')
            self.draw_text(1330, 500, 'Force Distribution')
            self.draw_text(373, 47, 'time (seconds)')
            glColor3f(0.1, 0.1, 0.1)
            self.draw_text(1400, 1040, 'Side Camera')
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(795, 80, '0s')
            self.draw_text(645, 80, '-1s')
            self.draw_text(505, 80, '-2s')
            self.draw_text(364, 80, '-3s')
            self.draw_text(223, 80, '-4s')
            self.draw_text(84, 80, '-5s')
            self.draw_text(44, 231, '0')
            self.draw_text(44, 531, '0')
            self.draw_text(44, 831, '0')
            self.draw_text(20, 107, '-100.0')
            self.draw_text(30, 357, '100.0')
            self.draw_text(20, 408, '-100.0')
            self.draw_text(30, 657, '100.0')
            self.draw_text(20, 708, '-100.0')
            self.draw_text(30, 957, '100.0')
            self.draw_text(1810, 451, '1000 N')
            self.draw_text(1810, 252, '500 N')
            self.draw_text(1810, 55, '0 N')
            glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == '5':
            glViewport(0, 0, self.width, self.height)

            # Render the two Camera colour images.
            if len(self.camera_color_size) > 0 and len(self.camera2_color_size) > 0:
                self.render_img(1400, 480, self.camera_color_img, self.camera_color_size[0], self.camera_color_size[1], 1, 1, 1, 0)
                self.render_img(880, 480, self.camera2_color_img, self.camera2_color_size[0], self.camera2_color_size[1], 1, 1, 1, 0)

            # Render the colorbar.
            self.render_img(1800, 50, self.rgb_colorbar, self.rgb_colorbar_size[0], self.rgb_colorbar_size[1], 1, 1, 1, 1)

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

            # Accelerometer (IMU) render.
            disp1, disp2, disp3 = self.time_series1.display(), self.time_series2.display(), self.time_series3.display()

            # Draw grid.
            grid1, grid2, grid3 = disp1['grid'], disp2['grid'], disp3['grid']
            glColor3f(0.1, 0.1, 0.1)
            glLineWidth(1.0)
            for i in grid1['thin'] + grid2['thin'] + grid3['thin']:
                self.draw_line(i)
            glColor3f(0.2, 0.2, 0.2)
            glLineWidth(2.0)
            for i in grid1['thick'] + grid2['thick'] + grid3['thick']:
                self.draw_line(i)

            # Draw data polylines.
            glColor3f(1.0, 0.0, 0.0)
            self.draw_line_strip(disp1['data'])
            self.draw_line_strip(disp2['data'])
            self.draw_line_strip(disp3['data'])

            # Draw axes.
            glColor3f(1.0, 1.0, 1.0)
            glLineWidth(3.0)
            ax1, ax2, ax3 = disp1['axes'], disp2['axes'], disp3['axes']
            self.draw_line(ax1[0])
            self.draw_line(ax1[1])
            self.draw_line(ax2[0])
            self.draw_line(ax2[1])
            self.draw_line(ax3[0])
            self.draw_line(ax3[1])

            # Mesh render.
            mesh_data = self.mesh_view.display()
            front, right, colors = mesh_data['front'], mesh_data['right'], mesh_data['colors']

            # Draw beams.
            glLineWidth(1.0)
            num_beams = len(front['beams'])
            for i in range(num_beams):
                color = colors[i]
                glColor3f(color[0], color[1], color[2])
                line = front['beams'][i]
                self.draw_line([line[0], line[1], line[2], line[3]])
                line = right['beams'][i]
                self.draw_line([line[0], line[1], line[2], line[3]])

            # Draw nodes.
            glColor3f(0.75, 0.75, 0.60)
            num_nodes = len(front['nodes'])
            for i in range(num_nodes):
                node = front['nodes'][i]
                glRectf(node[0] - 2, node[1] - 2, node[0] + 2, node[1] + 2)
                node = right['nodes'][i]
                glRectf(node[0] - 2, node[1] - 2, node[0] + 2, node[1] + 2)

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.65, 0.65, 0.3)
            self.draw_text(820, 233, 'X')
            self.draw_text(820, 534, 'Y')
            self.draw_text(820, 834, 'Z')
            self.draw_text(350, 985, 'IMU - Acceleration')
            self.draw_text(1065, 985, 'Tire: Front')
            self.draw_text(1580, 985, 'Tire: Side')
            self.draw_text(1170, 400, 'Vehicle Force Distribution')
            self.draw_text(373, 45, 'time (seconds)')
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(794, 80, '0s')
            self.draw_text(646, 80, '-1s')
            self.draw_text(506, 80, '-2s')
            self.draw_text(366, 80, '-3s')
            self.draw_text(225, 80, '-4s')
            self.draw_text(85, 80, '-5s')
            self.draw_text(44, 231, '0')
            self.draw_text(44, 531, '0')
            self.draw_text(44, 831, '0')
            self.draw_text(20, 107, '-18.0')
            self.draw_text(30, 357, '18.0')
            self.draw_text(20, 408, '-18.0')
            self.draw_text(30, 657, '18.0')
            self.draw_text(20, 708, '-18.0')
            self.draw_text(30, 957, '18.0')
            self.draw_text(1840, 456, '300 N')
            self.draw_text(1840, 257, '150 N')
            self.draw_text(1840, 60, '0 N')
            glDisable( GL_TEXTURE_2D )

            # Restore matrices.
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()

        elif self.demo == '6':

            # Render the two Camera colour images.
            if len(self.camera_color_size) > 0 and len(self.camera2_color_size) > 0:
                self.render_img(530, 10, self.camera_color_img, self.camera_color_size[0], self.camera_color_size[1], 1, 1, 1, 0)
                self.render_img(1150, 10, self.camera2_color_img, self.camera2_color_size[0], self.camera2_color_size[1], 1, 1, 1, 0)

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

            # Get display data.
            disp1, disp2, disp3 = self.time_series1.display(), self.time_series2.display(), self.time_series3.display()
            disp4, disp5, disp6 = self.time_series4.display(), self.time_series5.display(), self.time_series6.display()

            # Draw grid.
            grid1, grid2, grid3, grid4, grid5, grid6 = disp1['grid'], disp2['grid'], disp3['grid'], disp4['grid'], disp5['grid'], disp6['grid']
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
            self.draw_line_strip(disp1['data'])
            self.draw_line_strip(disp2['data'])
            self.draw_line_strip(disp3['data'])
            self.draw_line_strip(disp4['data'])
            self.draw_line_strip(disp5['data'])
            self.draw_line_strip(disp6['data'])

            # Draw axes.
            glColor3f(1.0, 1.0, 1.0)
            glLineWidth(3.0)
            ax1, ax2, ax3, ax4, ax5, ax6 = disp1['axes'], disp2['axes'], disp3['axes'], disp4['axes'], disp5['axes'], disp6['axes']
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

            # Draw final display markups.
            spike1_x, spike2_x, spike3_x, spike4_x, spike5_x, spike6_x = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            if self.is_final_display == True:
                glColor3f(1.0, 1.0, 0.0)
                glLineWidth(2.0)
                spike1_x = self.time_series1.find_first_spike(tol=0.15)
                spike2_x = self.time_series2.find_first_spike(tol=0.15)
                spike3_x = self.time_series3.find_first_spike(tol=0.15)
                spike4_x = self.time_series4.find_first_spike(tol=0.15)
                spike5_x = self.time_series5.find_first_spike(tol=0.15)
                spike6_x = self.time_series6.find_first_spike(tol=0.15)
                if spike4_x > spike1_x:
                    self.draw_line([spike1_x, 709, spike1_x, 1005])
                    self.draw_line([spike2_x, 709, spike2_x, 1005])
                    self.draw_line([spike3_x, 709, spike3_x, 1005])
                    self.draw_line([spike4_x, 420, spike4_x, 709])
                    self.draw_line([spike5_x, 420, spike5_x, 709])
                    self.draw_line([spike6_x, 420, spike6_x, 709])
                    glLineWidth(12.0)
                    glColor3f(1.0, 1.0, 0.05)
                    self.draw_line([spike1_x, 707, spike4_x, 707])
                    self.draw_line([spike2_x, 707, spike5_x, 707])
                    self.draw_line([spike3_x, 707, spike6_x, 707])

            # Draw Text.
            glEnable( GL_TEXTURE_2D )
            glBindTexture( GL_TEXTURE_2D, texid )
            glColor3f(0.65, 0.65, 0.3)
            self.draw_text(369, 1017, 'X')
            self.draw_text(1008, 1017, 'Y')
            self.draw_text(1621, 1017, 'Z')
            self.draw_text(307, 370, 'time (seconds)')
            self.draw_text(946, 370, 'time (seconds)')
            self.draw_text(1562, 370, 'time (seconds)')
            glColor3f(0.95, 0.25, 0.70)
            self.draw_text(10, 1040, '[Left IMU - Accel]')
            self.draw_text(10, 356, '[Right IMU - Accel]')
            glColor3f(0.85, 0.85, 0.70)
            self.draw_text(2, 436, '-300.0')
            self.draw_text(33, 561, '0')
            self.draw_text(7, 692, '300.0')
            self.draw_text(2, 736, '-300.0')
            self.draw_text(33, 863, '0')
            self.draw_text(7, 987, '300.0')

            self.draw_text(655, 408, '0s')
            self.draw_text(518, 408, '-0.05s')
            self.draw_text(404, 408, '-0.1s')
            self.draw_text(282, 408, '-0.15s')
            self.draw_text(177, 408, '-0.2s')
            self.draw_text(50, 408, '-0.25s')

            self.draw_text(1293, 408, '0s')
            self.draw_text(1162, 408, '-0.05s')
            self.draw_text(1045, 408, '-0.1s')
            self.draw_text(923, 408, '-0.15s')
            self.draw_text(812, 408, '-0.2s')
            self.draw_text(690, 408, '-0.25s')

            self.draw_text(1884, 408, '0s')
            self.draw_text(1764, 408, '-0.05s')
            self.draw_text(1657, 408, '-0.1s')
            self.draw_text(1545, 408, '-0.15s')
            self.draw_text(1441, 408, '-0.2s')
            self.draw_text(1330, 408, '-0.25s')

            if self.is_final_display == True and spike4_x > spike1_x:
                self.draw_text(spike4_x + 8, 706, 'LAG')
                self.draw_text(spike5_x + 8, 706, 'LAG')
                self.draw_text(spike6_x + 8, 706, 'LAG')
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