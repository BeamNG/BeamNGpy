from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Radar

def main():
    set_up_simple_logging()

    beamng = BeamNGpy('localhost', 64256)
    bng = beamng.open(launch=True)

    scenario = Scenario('west_coast_usa', 'RADAR_demo', description='Spanning the map with a RADAR sensor')

    vehicle = Vehicle('ego_vehicle', model='etk800', license='RED', color='Red')

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(bng)

    bng.settings.set_deterministic(60) # Set simulator to 60hz temporal resolution

    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # NOTE: Create sensor after scenario has started.
    radar = Radar('radar1', bng, vehicle, requested_update_time=0.01)

    vehicle.ai.set_mode('span')
    print('Driving around, polling the RADAR sensor every 5 seconds...')
    for i in range(1000):
        sleep(5)
        readings_data = radar.poll()
        print('Current RADAR readings (range, doppler velocity, azimuth (rad), elevation (rad), Radar Cross Section, Signal-to-Noise Ratio):')
        for j in range(20):
            print(readings_data[j])

    radar.remove()
    bng.close()

if __name__ == '__main__':
    main()
