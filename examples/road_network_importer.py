from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.tools import OpenDrive_Importer, OpenStreetMap_Importer, Sumo_Importer

def main():
    set_up_simple_logging()

    # Initialize BeamNG.
    print("Initializing BeamNG...")
    beamng = BeamNGpy('localhost', 64256)
    beamng.open(launch=True)
    scenario = Scenario('smallgrid', 'roads_importer')
    vehicle = Vehicle('ego_vehicle', model='etk800')
    scenario.add_vehicle(vehicle)

    # Import OpenDrive (.xodr).
    filename = 'Ex_LHT-Complex-X-Junction.xodr'
    OpenDrive_Importer.import_xodr(filename, scenario)                      # import an OpenDrive file (.xodr).

    # Import OpenStreetMap (.osm).
    #filename = 'map.osm'
    #OpenStreetMap_Importer.import_osm(filename, scenario)                  # import an OpenStreetMap file (.osm).

    #prefix = 'back'                                                        # Import Sumo files (.nod.xml, .edg.xml).
    #Sumo_Importer.import_sumo(prefix, scenario)

    # Start up BeamNG with the imported road network.
    print("Making scenario...")
    scenario.make(beamng)
    beamng.scenario.load(scenario)
    beamng.scenario.start()

    # Execute BeamNG until the user closes it.
    print("Completed.")
    while(True):
        pass
    beamng.close()


if __name__ == '__main__':
    main()
