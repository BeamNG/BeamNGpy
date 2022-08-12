import matplotlib.pyplot as plt
import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera

beamng = BeamNGpy('localhost', 64256)
beamng.open()

# Create a vehicle with a camera sensor attached to it
vehicle = Vehicle('ego_vehicle', model='etki', licence='PYTHON', color='Green')
cam_pos = np.array([0, 0, 0])
cam_dir = np.array([0, 0, 0])
cam_fov = 70
cam_res = (512, 512)
camera = Camera(cam_pos, cam_dir, cam_fov, cam_res, colour=True, annotation=True, depth=True)
vehicle.attach_sensor('camera', camera)

# Simple scenario with the vehicle we just created standing at the origin
car_pos = np.array([0, 0, 0])
scenario = Scenario('smallgrid', 'multishot', description='Demo of the camera sensor used like a multishot camera')
scenario.add_vehicle(vehicle, pos=car_pos)
scenario.make(beamng)

variations = [np.radians(a) for a in [-10, 0, 10]]

def get_bbox_center(bbox):
    rbl = np.array(bbox['rear_bottom_left'])
    fbr = np.array(bbox['front_bottom_right'])
    rtl = np.array(bbox['rear_top_left'])
    center = rbl + ((fbr - rbl) / 2) + ((rtl - rbl) / 2)
    return center

def rotate(vec, h, v):
    rot_x = np.cos(h) * vec[0] - np.sin(h) * vec[1]
    rot_y = np.sin(h) * vec[0] + np.cos(h) * vec[1]
    rot_z = np.sin(v) * rot_y + np.cos(v) * vec[2]
    return np.array([rot_x, rot_y, rot_z])


beamng.load_scenario(scenario)
beamng.start_scenario()
beamng.pause()

center = get_bbox_center(vehicle.get_bbox())
center[1] -= 0.75  # move center back a bit to get more of the vehicle
cam_off = center + np.array([3.5, 0, 0])  # Offset the camera 4 meters in front of the center

images = []
for h in variations:
    image_row = []
    for v in variations:
        current_pos = rotate(cam_off, h, v)
        current_dir = center - current_pos

        camera.pos = current_pos
        camera.direction = current_dir

        image = beamng.poll_sensors(vehicle)['camera']['colour']        # colour image.
        #image = beamng.poll_sensors(vehicle)['camera']['annotation']   # try this instead for the annotation image.
        #image = beamng.poll_sensors(vehicle)['camera']['depth']        # try this instead for the depth image.
        image_row.append(image)

    images.append(image_row)

beamng.close()

# Plot the data as images.
fig, ax = plt.subplots(3, 3, figsize=(30, 30))
for x, row in enumerate(images):
    for y, image in enumerate(row):
        ax[y, x].imshow(np.asarray(image.convert('RGB')))
        ax[y, x].set_aspect('equal', 'datalim')
plt.show()