"""
.. module:: west_coast_lidar
    :platform: Windows
    :synopsis: Example code making a scenario in west_coast_usa and having the
               vehicle span the map while emitting Lidar.
.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
"""
import random
from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Lidar


def main():
    random.seed(1703)
    set_up_simple_logging()

    beamng = BeamNGpy('localhost', 64256)
    bng = beamng.open(launch=True)

    scenario = Scenario('west_coast_usa', 'LiDAR_demo', description='Spanning the map with a LiDAR sensor')

    vehicle = Vehicle('ego_vehicle', model='etk800', license='RED', color='Red')

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675),
                         rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(bng)

    bng.settings.set_deterministic(60) # Set simulator to 60hz temporal resolution

    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # NOTE: Create sensor after scenario has started.
    # lidar = Lidar('lidar1', bng, vehicle, requested_update_time=0.01, is_using_shared_memory=True)     # Send data via shared memory.
    lidar = Lidar('lidar1', bng, vehicle, requested_update_time=0.01, is_using_shared_memory=False)   # Send data through lua socket instead.

    vehicle.ai.set_mode('span')
    print('Driving around, polling the LiDAR sensor every 5 seconds...')
    for i in range(100):
        sleep(5)
        readings_data = lidar.poll() # Fetch the latest readings from the sensor, over either shared memory or the lua socket.
        print('LiDAR Point Cloud Data after ', i * 5, ' seconds: ', readings_data['pointCloud'][0:10])       # The first 10 points from LiDAR point cloud data.
        # print('LiDAR Colour Data after ', i * 5, ' seconds: ', readings_data['colours'][0:10])             # The colour data (corresponds to each point).

    lidar.remove()
    bng.close()


if __name__ == '__main__':
    main()
