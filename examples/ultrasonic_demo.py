"""
.. module:: west_coast_random
    :platform: Windows
    :synopsis: Using an ultrasonic parking sensor on the vehicle front bumper.
.. moduleauthor:: Dave Stark <dstark@beamng.gmbh>
"""
import random

from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.sensors import Ultrasonic


def main():
    random.seed(1703)
    setup_logging()

    beamng = BeamNGpy('localhost', 64256)
    bng = beamng.open(launch=True)

    scenario = Scenario('west_coast_usa', 'ultrasonic_demo', description='Spanning the map with an ultrasonic sensor')

    vehicle = Vehicle('ego_vehicle', model='etk800', licence='RED', color='Red')

    ultrasonic = Ultrasonic(isSnappingDesired=True, isForceInsideTriangle=True)
    vehicle.attach_sensor('ultrasonic', ultrasonic)

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675),
                         rot=None, rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(bng)

    try:
        bng.set_deterministic()  # Set simulator to be deterministic
        bng.set_steps_per_second(60)  # With 60hz temporal resolution

        bng.load_scenario(scenario)
        bng.hide_hud()
        bng.start_scenario()

        #vehicle.ai_set_mode('span')
        print('Driving around for 60 seconds...')
        sleep(60)
    finally:
        bng.close()


if __name__ == '__main__':
    main()