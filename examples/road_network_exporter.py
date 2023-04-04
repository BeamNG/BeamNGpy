from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.tools import RoadNetworkExporter


def main():
    set_up_simple_logging()

    bng = BeamNGpy('localhost', 64256)
    bng.open(launch=True)
    scenario = Scenario('west_coast_usa', 'Road Network Exporter Demo', description='Exports map data')
    vehicle = Vehicle('ego_vehicle', model='etk800', license='RED', color='Red')
    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(bng)
    bng.settings.set_deterministic(60)  # Set simulator to 60hz temporal resolution
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # Get the road graph data for the map.
    graph = RoadNetworkExporter(bng)
    path_segments = graph.compute_path_segments()
    graph.plot_path_segments(path_segments)             # Plots the road data with Matplotlib.
    graph.export_xodr('test_od')                        # export to OpenDrive (.xodr) format.
    # graph.export_osm('test_od')                        # export to OpenStreetMap (.osm) format.

    vehicle.ai.set_mode('span')
    for _ in range(10):
        sleep(1.0)

    bng.close()


if __name__ == '__main__':
    main()
