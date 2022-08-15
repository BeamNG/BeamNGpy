import random
from time import sleep
from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Accelerometer

def main():
    random.seed(1703)
    set_up_simple_logging()

    beamng = BeamNGpy('localhost', 64256)
    bng = beamng.open(launch=True)

    scenario = Scenario('west_coast_usa', 'accelerometer_demo', description='Spanning the map with an accelerometer sensor')

    vehicle = Vehicle('ego_vehicle', model='etk800', licence='RED', color='Red')

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(bng)

    bng.set_deterministic()
    bng.set_steps_per_second(60)  # Set simulator to 60hz temporal resolution

    bng.load_scenario(scenario)
    bng.hide_hud()
    bng.start_scenario()

    # NOTE: Create sensor after scenario has started.
    accel = Accelerometer('accel1', bng, vehicle, requested_update_time=0.01)
    #accel = Accelerometer('accel1', bng, vehicle, requested_update_time=0.01, pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1)) # if we want to specify a local frame

    vehicle.ai_set_mode('span')
    print('Driving around, polling the accelerometer sensor at regular intervals...')
    for i in range(10000):
        sleep(0.1) # Include a small delay between each reading.
        data = accel.poll() # Fetch the latest readings from the sensor.
        print("Acceleration in each axis of the sensor: ", data)

    accel.remove()
    bng.close()


if __name__ == '__main__':
    main()