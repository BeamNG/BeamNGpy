"""
.. module:: east_coast_random
    :platform: Windows
    :synopsis: Example code making a scenario in east_coast_usa and having a
               car drive around randomly.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""
import random

from matplotlib import pyplot as plt

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera, Damage, Electrics, GForces, Timer


def main():
    random.seed(1703)

    set_up_simple_logging()

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
    bng = beamng.open(launch=True)

    # Create a scenario in east_coast_usa
    scenario = Scenario('east_coast_usa', 'tech_test', description='Random driving for research')

    # Set up first vehicle, with two cameras, gforces sensor, lidar, electrical
    # sensors, and damage sensors
    vehicle = Vehicle('ego_vehicle', model='etk800', license='RED', color='Red')

    # Set up sensors
    gforces = GForces()
    electrics = Electrics()
    damage = Damage()
    timer = Timer()

    # Attach them
    vehicle.attach_sensor('gforces', gforces)
    vehicle.attach_sensor('electrics', electrics)
    vehicle.attach_sensor('damage', damage)
    vehicle.attach_sensor('timer', timer)

    scenario.add_vehicle(vehicle, pos=(-426.68, -43.59, 31.11), rot_quat=(0, 0, 1, 0))

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(bng)

    # Start BeamNG and enter the main loop
    try:
        bng.ui.hide_hud()
        bng.settings.set_deterministic(60)  # Set simulator to be deterministic, with 60 Hz temporal resolution

        # Load and start the scenario
        bng.scenario.load(scenario)
        bng.scenario.start()
        # Put simulator in pause awaiting further inputs
        bng.control.pause()

        assert vehicle.is_connected()

        pos = (0.0, -3, 1)
        direction = (0, -1, 0)
        fov = 120
        resolution = (512, 512)
        front_camera = Camera('front_camera', bng, vehicle,
            pos=pos, dir=direction, field_of_view_y=fov, resolution=resolution,
            is_render_colours=True, is_render_depth=True, is_render_annotations=True)

        pos = (0.0, 3, 1.0)
        direction = (0, 1, 0)
        fov = 90
        resolution = (512, 512)
        back_camera = Camera('back_camera', bng, vehicle,
            pos=pos, dir=direction, field_of_view_y=fov, resolution=resolution,
            is_render_colours=True, is_render_depth=True, is_render_annotations=True)

        # Send random inputs to vehice and advance the simulation 20 steps
        for _ in range(1024):
            throttle = random.uniform(0.0, 1.0)
            steering = random.uniform(-1.0, 1.0)
            brake = random.choice([0, 0, 0, 1])
            vehicle.control(throttle=throttle, steering=steering, brake=brake)

            bng.control.step(20)

            # Retrieve sensor data and show the camera data.
            vehicle.sensors.poll()
            sensors = vehicle.sensors

            print('{} seconds passed.'.format(sensors['timer']['time']))

            front_cam_data = front_camera.poll()
            back_cam_data = back_camera.poll()

            a_colour.imshow(front_cam_data['colour'].convert('RGB'))
            a_depth.imshow(front_cam_data['depth'].convert('L'))
            a_annot.imshow(front_cam_data['annotation'].convert('RGB'))

            b_colour.imshow(back_cam_data['colour'].convert('RGB'))
            b_depth.imshow(back_cam_data['depth'].convert('L'))
            b_annot.imshow(back_cam_data['annotation'].convert('RGB'))

            plt.pause(1.0)
    finally:
        bng.close()


if __name__ == '__main__':
    main()
