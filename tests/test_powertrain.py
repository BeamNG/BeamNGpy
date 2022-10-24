from time import sleep
from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import PowertrainSensor

# Executing this file will perform various tests on all available functionality relating to the powertrain sensor.
# It is provided to give examples on how to use all powertrain sensor functions currently available in beamngpy.

if __name__ == '__main__':
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy('localhost', 64256)
    bng.open(launch=True)
    vehicle = Vehicle('ego_vehicle', model='etki', licence='PYTHON', color='Red')                           # Create a vehicle.
    scenario = Scenario('smallgrid', 'powertrain_test', description='Testing the powertrain sensor')        # Create a scenario.
    scenario.add_vehicle(vehicle)                                                                           # Add the vehicle to the scenario.
    scenario.make(bng)
    bng.set_deterministic()
    bng.set_steps_per_second(60)                                                                            # Set simulator to 60hz temporal resolution
    bng.load_scenario(scenario)
    bng.hide_hud()
    bng.start_scenario()

    print("Powertrain test start.")

    # Create a default powertrain sensor.
    powertrain1 = PowertrainSensor('powertrain1', bng, vehicle, is_send_immediately=True)

    # Test the automatic polling functionality of the powertrain sensor, to make sure we retrieve the readings data via shared memory.
    sleep(2)
    sensor_readings = powertrain1.poll()
    print("powertrain readings (automatic polling): ", sensor_readings)

    # Test the ad-hoc polling functionality of the powertrain sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
    sleep(1)
    print("Ad-hoc poll request test.")
    request_id = powertrain1.send_ad_hoc_poll_request()                                                  # send an ad-hoc polling request to the simulator.
    print("Ad-hoc poll requests sent. Unique request Id number: ", request_id)
    sleep(3)
    print("Is ad-hoc request complete? ", powertrain1.is_ad_hoc_poll_request_ready(request_id))          # Ensure that the data has been processed before collecting.
    sensor_readings_ad_hoc = powertrain1.collect_ad_hoc_poll_request(request_id)                         # Collect the data now that it has been computed.
    print("powertrain readings (ad-hoc polling): ", sensor_readings_ad_hoc)
    powertrain1.remove()
    print("powertrain sensor removed.")

    sleep(3)
    print("Powertrain test complete.")

    # Close the simulation.
    bng.close()
