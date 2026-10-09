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
    OpenDriveExporter.export("west_coast_usa", bng)

    # Export OpenStreetMap (.osm).
    OpenStreetMapExporter.export('west_coast_usa', bng)

    # Export SUMO (.nod.xml and .edg.xml) into the BeamNG.tech user folder
    # (typically <userpath>/BeamNG.tech/current/).
    SumoExporter.export("west_coast_usa", bng)

    # Validate the exported network in SUMO (cmd, from that folder).
    # This checks that the network loads and can carry traffic; it is not a
    # live BeamNG–SUMO co-simulation.
    #
    #   netconvert --node-files=west_coast_usa.nod.xml --edge-files=west_coast_usa.edg.xml -o west_coast_usa.net.xml
    #   python "%SUMO_HOME%\tools\randomTrips.py" -n west_coast_usa.net.xml -r west_coast_usa.rou.xml -e 3600 -p 2
    #   sumo-gui -n west_coast_usa.net.xml -r west_coast_usa.rou.xml
    #
    # netconvert may print warnings and still succeed (look for "Success.").
    # On dense junctions, native netconvert options (or a .netccfg via
    # netconvert -c) reduce stub edges: --junctions.join --junctions.join-dist 15
    # --default.junctions.radius 1.5
    # Wait until randomTrips.py finishes before launching sumo-gui, then press
    # Play. Prefer an official map such as west_coast_usa for the first test.

    # Execute the simulation until user is finished.  The data file(s) have been written.
    print("Road network data exported.")
    bng.ui.show_hud()
    input("Press Enter to exit...")
    bng.disconnect()


if __name__ == "__main__":
    main()
