import random
from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import AdvancedIMU


def main():
    random.seed(1703)
    set_up_simple_logging()

    beamng = BeamNGpy('localhost', 64256)
    bng = beamng.open(launch=True)

    scenario = Scenario('west_coast_usa', 'advanced_IMU_demo', description='Spanning the map with an advanced IMU sensor')
    vehicle = Vehicle('ego_vehicle', model='etk800', license='RED', color='Red')

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(bng)

    bng.settings.set_deterministic(60) # Set simulator to 60hz temporal resolution

    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # NOTE: Create sensor after scenario has started.
    IMU = AdvancedIMU('accel1', bng, vehicle, gfx_update_time=0.01)
    #IMU = AdvancedIMU('accel1', bng, vehicle, gfx_update_time=0.01, pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1)) # if we want to specify a local frame

    vehicle.ai.set_mode('span')
    print('Driving around, polling the advanced IMU sensor at regular intervals...')
    for i in range(10000):
        sleep(0.1) # Include a small delay between each reading.
        data = IMU.poll() # Fetch the latest readings from the sensor.
        print('Acceleration in each axis of the sensor: ', data)

    IMU.remove()
    bng.close()


if __name__ == '__main__':
    main()