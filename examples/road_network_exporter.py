from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.tools import OpenDriveExporter, OpenStreetMapExporter, SumoExporter


def main():
    set_up_simple_logging()

    bng = BeamNGpy("localhost", 25252)
    bng.open(launch=True)
    scenario = Scenario(
        "west_coast_usa", "Road Network Exporter Demo", description="Exports map data"
    )
    vehicle = Vehicle("ego_vehicle", model="etk800", license="RED", color="Red")
    scenario.add_vehicle(
        vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795)
    )
    scenario.make(bng)
    bng.settings.set_deterministic(60)
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    print("Exporting road network data...")

    # Export OpenDrive (.xodr).
    OpenDriveExporter.export("test_od", bng)

    # Export OpenStreetMap (.osm).
    # OpenStreetMapExporter.export('test_osm', bng)

    # Export Sumo (.nod.xml and .edg.xml).
    # SumoExporter.export('test', bng)

    # Execute the simulation until user is finished.  The data file(s) have been written.
    print("Road network data exported.")
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
