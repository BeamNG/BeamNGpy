from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Radar

import seaborn as sns
sns.set()  # Let seaborn apply better styling to all matplotlib graphs

def main():
    set_up_simple_logging()

    beamng = BeamNGpy('localhost', 64256)
    bng = beamng.open(launch=True)

    scenario = Scenario('gridmap', 'RADAR_demo', description='Spanning the map with a RADAR sensor')

    vehicle = Vehicle('ego_vehicle', model='etk800', license='RED', color='Red')

    scenario.add_vehicle(vehicle)
    scenario.make(bng)

    bng.settings.set_deterministic(60) # Set simulator to 60hz temporal resolution

    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # NOTE: Create sensor after scenario has started.
    RANGE_MIN = 0.1
    RANGE_MAX = 100.0
    RESOLUTION = (200, 200)
    FOV = 70
    radar = Radar('radar1', bng, vehicle,
        requested_update_time=0.01,
        pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1),
        resolution=RESOLUTION, field_of_view_y=FOV, near_far_planes=(RANGE_MIN, RANGE_MAX),
        range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12, range_min_cutoff=0.5, range_direct_max_cutoff=RANGE_MAX)

    for _ in range(1000):
        sleep(5)
        readings_data = radar.poll() # Fetch the latest readings from the sensor, over either shared memory or the lua socket.
        radar.plot_data(readings_data, RESOLUTION, FOV, RANGE_MIN, RANGE_MAX, 200, 200)

    radar.remove()
    bng.close()

if __name__ == '__main__':
    main()
