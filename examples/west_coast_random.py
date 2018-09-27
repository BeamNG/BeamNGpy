"""
.. module:: west_coast_random
    :platform: Windows
    :synopsis: Example code making a scenario in west_coast_usa and having two
               cars drive around randomly.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""
import mmap
import random

from time import sleep

from matplotlib import pyplot as plt
from matplotlib.pyplot import imshow

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.sensors import Camera, GForces, Lidar, Electrics, Damage


def main():
    random.seed(1703)

    setup_logging()

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

    # Create a scenario in west_coast_usa
    scenario = Scenario('west_coast_usa', 'research_test',
                        description='Random driving for research')

    # Set up first vehicle, with a frontal camera, gforces sensor, and lidar.
    vehicle = Vehicle('ego_vehicle', model='etk800',
                      licence='FOO', color='Red')

    # Set up sensors
    pos = (-0.3, 1, 1.0)
    direction = (0, 1, 0)
    fov = 120
    resolution = (512, 512)
    camera = Camera(pos, direction, fov, resolution,
                    colour=True, depth=True, annotation=True)
    vehicle.attach_sensor('front_cam', camera)
    gforces = GForces()
    vehicle.attach_sensor('gforces', gforces)
    lidar = Lidar()
    vehicle.attach_sensor('lidar', lidar)

    # Add the vehicle to the scenario
    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675),
                         rot=(0, 0, 45))

    # Set up the second vehicle, with a frontal camera, gforces sensor,
    # electrics, and damage sensors.
    other_vehicle = Vehicle('other_vehicle', model='etk800',
                            licence='BAR', color='Blue')
    # Set up sensors
    pos = (-0.3, 1, 1.0)
    direction = (0, 1, 0)
    fov = 60
    resolution = (512, 512)
    camera = Camera(pos, direction, fov, resolution,
                    colour=True, depth=True, annotation=True)

    gforces = GForces()
    electrics = Electrics()
    damage = Damage()

    # Attach them
    other_vehicle.attach_sensor('front_cam', camera)
    other_vehicle.attach_sensor('gforces', gforces)
    other_vehicle.attach_sensor('electrics', electrics)
    other_vehicle.attach_sensor('damage', damage)

    # Add vehicle to scenario
    scenario.add_vehicle(other_vehicle, pos=(-722, 101, 118.675),
                         rot=(0, 0, 45))

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)

    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)
    try:
        bng.set_deterministic()  # Set simulator to be deterministic
        bng.set_steps_per_second(60)  # With 60hz temporal resolution

        # Load and start the scenario
        bng.load_scenario(scenario)
        bng.start_scenario()
        # Put simulator in pause awaiting further inputs
        bng.pause()
        bng.hide_hud()

        assert vehicle.skt
        assert other_vehicle.skt

        # Send random inputs to both vehices and advance the simulation 6 steps
        for _ in range(1024):
            throttle = random.uniform(0.0, 1.0)
            steering = random.uniform(-1.0, 1.0)
            brake = random.choice([0, 0, 0, 1])
            vehicle.control(throttle=throttle, steering=steering, brake=brake)

            throttle = random.uniform(0.0, 1.0)
            steering = random.uniform(-1.0, 1.0)
            brake = random.choice([0, 0, 0, 1])
            other_vehicle.control(throttle=throttle, steering=steering,
                                  brake=brake)

            bng.step(6)

            # Retrieve sensor data and show the camera data.
            sensors = bng.poll_sensors(vehicle)
            a_colour.imshow(sensors['front_cam']['colour'].convert('RGB'))
            a_depth.imshow(sensors['front_cam']['depth'].convert('L'))
            a_annot.imshow(sensors['front_cam']['annotation'].convert('RGB'))

            sensors = bng.poll_sensors(other_vehicle)
            b_colour.imshow(sensors['front_cam']['colour'].convert('RGB'))
            b_depth.imshow(sensors['front_cam']['depth'].convert('L'))
            b_annot.imshow(sensors['front_cam']['annotation'].convert('RGB'))

            plt.pause(0.0016)
    finally:
        bng.close()


if __name__ == '__main__':
    main()
