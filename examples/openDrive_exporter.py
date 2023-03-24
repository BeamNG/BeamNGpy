from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Road_Graph

def main():
    set_up_simple_logging()

    bng = BeamNGpy('localhost', 64256)
    bng.open(launch=True)
    scenario = Scenario('west_coast_usa', 'OpenDrive_Exporter_Demo', description='Exports map to OpenDrive .xodr format')
    vehicle = Vehicle('ego_vehicle', model='etk800', license='RED', color='Red')
    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(bng)
    bng.settings.set_deterministic(60) # Set simulator to 60hz temporal resolution
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # Get the road graph data for the map.
    graph = Road_Graph(bng)
    path_segments = graph.compute_path_segments()
    graph.plot_path_segments(path_segments)
    graph.export_xodr('test_od')

    vehicle.ai.set_mode('span')
    for _ in range(100000):
        sleep(15)

    bng.close()


if __name__ == '__main__':
    main()
