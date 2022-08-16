from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Accelerometer

# Executing this file will perform various tests on all available functionality relating to the accelerometer sensor.
# It is provided to give examples on how to use all accelerometer sensor functions currently available in beamngpy.

if __name__ == '__main__':
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy('localhost', 64256)
    bng.open(launch=True)

    # Create a vehicle.
    vehicle = Vehicle('ego_vehicle', model='etki', licence='PYTHON', color='Red')
    # Create a scenario.
    scenario = Scenario('smallgrid', 'accelerometer_test', description='Testing the accelerometer sensor')
    # Add the vehicle to the scenario.
    scenario.add_vehicle(vehicle)
    scenario.make(bng)

    bng.set_deterministic()
    bng.set_steps_per_second(60)        # Set simulator to 60hz temporal resolution

    bng.load_scenario(scenario)
    bng.hide_hud()
    bng.start_scenario()

    print("Accelerometer test start.")

    # Create a default accelerometer sensor.
    accel1 = Accelerometer('accelerometer1', bng, vehicle)

    # Test the automatic polling functionality of the accelerometer sensor, to make sure we retrieve the point cloud data via shared memory.
    sleep(2)
    sensor_readings = accel1.poll()
    print("Accelerometer readings (automatic polling): ", sensor_readings)

    # Test the ad-hoc polling functionality of the accelerometer sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
    sleep(1)
    print("Ad-hoc poll request test.")
    # send an ad-hoc polling request to the simulator.
    request_id = accel1.send_ad_hoc_poll_request()
    print("Ad-hoc poll requests sent. Unique request Id number: ", request_id)
    sleep(3)
    # Ensure that the data has been processed before collecting.
    print("Is ad-hoc request complete? ", accel1.is_ad_hoc_poll_request_ready(request_id)['data'])
    # Collect the data now that it has been computed.
    sensor_readings_ad_hoc = accel1.collect_ad_hoc_poll_request(request_id)
    print("Accelerometer readings (ad-hoc polling): ", sensor_readings_ad_hoc)
    accel1.remove()
    print("Accelerometer sensor removed.")

    # Create an accelerometer sensor which has a negative requested update rate, and ensure that no readings are computed from it.
    accel2 = Accelerometer('accelerometer2', bng, vehicle, requested_update_time=-1.0)
    print("Testing an acceleromter sensor with a negative requested update time...")
    sleep(2)
    sensor_readings = accel2.poll()
    print("Accelerometer readings (should be zeros): ", sensor_readings)
    accel2.remove()

    # Recreate the first accelerometer sensor.
    accel1 = Accelerometer('accelerometer1', bng, vehicle)

    # Test that the property getter function return the correct data which was set.
    sleep(1)
    print("Property getter test.  The displayed values should be the values which were set during the creation of the accelerometer sensor.")
    print("Sensor Name: ", accel1.name)
    print("Position: ", accel1.get_position())
    print("Direction: ", accel1.get_direction())

    # Test changing the visibility of the sensor.
    print("Test visibility mode.  Accelerometer visibility should cycle between on and off 3 times, staying at each for 1 second.")
    sleep(1)
    accel1.set_is_visualised(False)
    sleep(1)
    accel1.set_is_visualised(True)
    sleep(1)
    accel1.set_is_visualised(False)
    sleep(1)
    accel1.set_is_visualised(True)
    sleep(1)
    accel1.set_is_visualised(False)
    sleep(1)
    accel1.set_is_visualised(True)

    accel1.remove()

    sleep(3)
    print("Accelerometer test complete.")

    # Close the simulation.
    bng.close()
