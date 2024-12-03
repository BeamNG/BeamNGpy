from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.tools import OpenDriveImporter, OpenStreetMapImporter, SumoImporter

def main():
    set_up_simple_logging()

    # Initialize BeamNG.
    print("Initializing BeamNG...")
    beamng = BeamNGpy("localhost", 25252)
    beamng.open(launch=True)
    scenario = Scenario("tech_ground", "roads_importer")
    vehicle = Vehicle("ego_vehicle", model="etk800")
    scenario.add_vehicle(vehicle)

    # Import OpenDrive (.xodr).
    filename = "data/Ex_LHT-Complex-X-Junction.xodr"
    OpenDriveImporter.import_xodr(
        filename, scenario
    )  # import an OpenDrive file (.xodr).

    # Import OpenStreetMap (.osm).
    # filename = 'map.osm'
    # OpenStreetMapImporter.import_osm(filename, scenario)                 # import an OpenStreetMap file (.osm).

    # prefix = 'back'                                                      # Import Sumo files (.nod.xml, .edg.xml).
    # SumoImporter.import_sumo(prefix, scenario)

    # Start up BeamNG with the imported road network.
    print("Making scenario...")
    scenario.make(beamng)
    beamng.scenario.load(scenario)
    beamng.scenario.start()

    # Execute BeamNG until the user closes it.
    input("Completed. Press Enter to exit...")


if __name__ == "__main__":
    main()
