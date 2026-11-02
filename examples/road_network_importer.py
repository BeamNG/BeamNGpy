from pathlib import Path

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.tools import OpenDriveImporter, OpenStreetMapImporter, SumoImporter

SCRIPT_DIR = Path(__file__).parent.resolve()


def choose_importer() -> str:
    """Ask which road-network format to import and return the menu choice."""
    print("Choose the road network format to import:")
    print("  1: OpenDRIVE (.xodr)")
    print("  2: OpenStreetMap (.osm)")
    print("  3: SUMO (.nod.xml + .edg.xml)")
    while True:
        choice = input("Enter 1-3 [1]: ").strip()
        if not choice:
            return "1"
        if choice in ("1", "2", "3"):
            return choice
        print("Invalid choice; please enter 1, 2 or 3.")


def main():
    set_up_simple_logging()

    # Pick a single importer up front. Only that one network is imported;
    # importing every format at once is too heavy on the simulator.
    choice = choose_importer()

    # Initialize BeamNG.
    print("Initializing BeamNG...")
    beamng = BeamNGpy("localhost", 25252)
    beamng.open(launch=True)
    scenario = Scenario("tech_ground", "roads_importer")
    vehicle = Vehicle("ego_vehicle", model="etk800")
    scenario.add_vehicle(vehicle)

    print("Making scenario...")
    scenario.make(beamng)
    beamng.scenario.load(scenario)
    beamng.scenario.start()

    # Import into the loaded level with the chosen importer. There is no need
    # to open the World Editor: Road Architect is initialized automatically
    # and the imported roads are finalized headlessly, ready to drive on.
    importers = {
        "1": (
            "OpenDRIVE",
            OpenDriveImporter(beamng).import_xodr,
            SCRIPT_DIR / "data/Ex_LHT-Complex-X-Junction.xodr",
        ),
        "2": (
            "OpenStreetMap",
            OpenStreetMapImporter(beamng).import_osm,
            SCRIPT_DIR / "data/London.osm",
        ),
        "3": (
            "SUMO",
            SumoImporter(beamng).import_sumo,
            # Generated from data/London.osm; see the SumoImporter docs for the
            # netconvert command.
            SCRIPT_DIR / "data/London",
        ),
    }
    label, import_network, path = importers[choice]
    print(f"Importing {label} road network from {path}...")

    # Import options, mirroring the editor 'Import Options' window:
    #   Placement (XY):  None = player vehicle pose (default) | (x, y) = custom
    #                    world position | "as_in_file" | "camera_position"
    #   Offset (Z):      offset_to_terrain=True OR custom_offset=<meters> —
    #                    mutually exclusive, both off by default
    #   Terraform:       terraform=True ('Terraform Terrain To Import'), with
    #                    domain_of_influence (default 30 m) and margin
    #                    (default 1.0 m)
    import_options = dict(
        # placement_xy=(300.0, 700.0),  # place the network at this world position
        # placement_xy="as_in_file",  # place the network at the same position as in the file
        placement_xy="camera_position",  # place the network at the camera position
        terraform=True,               # terraform the terrain to fit the network
        domain_of_influence=30.0,     # default: 30
        margin=1.0,                 # default: 1.0
    )
    result = import_network(path, **import_options)
    print(
        f"{label}: imported {result['importedRoadCount']} Road Architect "
        f"road(s); skipped {result['skippedRoadCount']}."
    )
    for warning in result["warnings"]:
        print(f"Warning: {warning}")

    # Execute BeamNG until the user closes it.
    input("Completed. Press Enter to exit...")
    beamng.disconnect()


if __name__ == "__main__":
    main()