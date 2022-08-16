"""
.. module:: east_coast_random
    :platform: Windows
    :synopsis: Example code making a scenario in west_coast_usa and having a
               car drive around randomly.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""
import random

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera, Damage, Electrics, GForces, Lidar, Timer
from matplotlib import pyplot as plt


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
    scenario = Scenario('east_coast_usa', 'tech_test',
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
    lidar = Lidar(is_visualised=False)
    timer = Timer()

    # Attach them
    vehicle.attach_sensor('front_cam', front_camera)
    vehicle.attach_sensor('back_cam', back_camera)
    vehicle.attach_sensor('gforces', gforces)
    vehicle.attach_sensor('electrics', electrics)
    vehicle.attach_sensor('damage', damage)
    vehicle.attach_sensor('timer', timer)

    scenario.add_vehicle(vehicle, pos=(900.648, -226.267, 40.285),
                         rot_quat=(0, 0, 0.3826834, 0.9238795))

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(bng)

    # Start BeamNG and enter the main loop
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
            vehicle.poll_sensors()
            sensors = vehicle.sensors

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
