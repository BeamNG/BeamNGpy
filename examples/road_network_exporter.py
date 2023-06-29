from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.tools import OpenDrive_Exporter, OpenStreetMap_Exporter

def main():
    set_up_simple_logging()

    bng = BeamNGpy('localhost', 64256)
    bng.open(launch=True)
    scenario = Scenario('west_coast_usa', 'Road Network Exporter Demo', description='Exports map data')
    vehicle = Vehicle('ego_vehicle', model='etk800', license='RED', color='Red')
    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(bng)
    bng.settings.set_deterministic(60)
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # Export OpenDrive (.xodr).
    OpenDrive_Exporter.export('test_od', bng)                   # export to OpenDrive (.xodr) format.

    # Export OpenStreetMap (.osm).
    #OpenStreetMap_Exporter.export('test_osm', bng)

    # Execute the simulation until user is finished.  The data file(s) have been written.
    vehicle.ai.set_mode('span')
    while(True):
        pass
    bng.close()


if __name__ == '__main__':
    main()
