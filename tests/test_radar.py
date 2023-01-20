from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Radar

# Executing this file will perform various tests on all available functionality relating to the RADAR sensor.
# It is provided to give examples on how to use all RADAR sensor functions currently available in beamngpy.

if __name__ == '__main__':
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy('localhost', 64256)
    bng.open(launch=True)
    vehicle = Vehicle('ego_vehicle', model='etki', licence='PYTHON', color='Red')                       # Create a vehicle.
    scenario = Scenario('smallgrid', 'radar_test', description='Testing the RADAR sensor')              # Create a scenario.
    scenario.add_vehicle(vehicle)                                                                       # Add the vehicle to the scenario.
    scenario.make(bng)
    bng.settings.set_deterministic(60)                                                                  # Set simulator to 60hz temporal resolution
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    print('RADAR test start.')

    # Create a default RADAR sensor.
    radar1 = Radar('radar1', bng, vehicle)

    # Test the automatic polling functionality of the RADAR sensor, to make sure we retrieve the readings.
    sleep(2)
    sensor_readings = radar1.poll()
    print('RADAR readings (automatic polling): ', sensor_readings)

    # Test the ad-hoc polling functionality of the RADAR sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
    sleep(1)
    print('Ad-hoc poll request test.')
    request_id = radar1.send_ad_hoc_poll_request()                                                          # send an ad-hoc polling request to the simulator.
    print('Ad-hoc poll requests sent. Unique request Id number: ', request_id)
    sleep(3)
    print('Is ad-hoc request complete? ', radar1.is_ad_hoc_poll_request_ready(request_id))                  # Ensure that the data has been processed before collecting.
    sensor_readings_ad_hoc = radar1.collect_ad_hoc_poll_request(request_id)                                 # Collect the data now that it has been computed.
    print('RADAR readings (ad-hoc polling): ', sensor_readings_ad_hoc)
    radar1.remove()
    print('RADAR sensor removed.')

    # Create a RADAR sensor which has a negative requested update rate, and ensure that no readings are computed from it.
    radar2 = Radar('radar2', bng, vehicle, requested_update_time=-1.0)
    print('Testing an ultrasonic sensor with a negative requested update time...')
    sleep(2)
    sensor_readings = radar2.poll()
    print('RADAR readings (should be zeros): ', sensor_readings)
    radar2.remove()

    # Recreate the first RADAR sensor.
    radar1 = Radar('radar1', bng, vehicle)

    # Test that the property getter function return the correct data which was set.
    sleep(1)
    print('Property getter test.  The displayed values should be the values which were set during the creation of the ultrasonic sensors.')
    print('Sensor Name: ', radar1.name)
    print('Position: ', radar1.get_position())
    print('Direction: ', radar1.get_direction())
    print('Requested update time: ', radar1.get_requested_update_time())
    print('Priority: ', radar1.get_update_priority())
    print('Max Pending Requests: ', radar1.get_max_pending_requests())

    # Test that we can set the sensor core properties in the simulator from beamngpy.
    sleep(1)
    print('Property setter test.  The displayed property values should be different from the previous values.')
    radar1.set_requested_update_time(0.3)
    print('Newly-set Requested Update Time: ', radar1.get_requested_update_time())
    radar1.set_update_priority(0.5)
    print('Newly-set Priority: ', radar1.get_update_priority())
    radar1.set_max_pending_requests(5)
    print('Newly-set Max Pending Requests: ', radar1.get_max_pending_requests())

    radar1.remove()

    sleep(3)
    print('RADAR test complete.')

    # Close the simulation.
    bng.close()
