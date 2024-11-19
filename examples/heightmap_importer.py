from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.tools import Terrain_Importer


def main():
    set_up_simple_logging()

    # Initialize BeamNG.
    print("Initializing BeamNG...")
    beamng = BeamNGpy("localhost", 25252)
    beamng.open(launch=True)
    scenario = Scenario("template_tech", "heightmap_importer")
    vehicle = Vehicle("ego_vehicle", model="etk800")
    scenario.add_vehicle(vehicle)

    # Start up BeamNG.
    print("Making scenario...")
    scenario.make(beamng)
    beamng.scenario.load(scenario)
    beamng.scenario.start()

    # Set the core terrain properties.
    w, h = 1000, 1000  # The width and height of the terrain.
    scale = 1  # The scale of the data (in metres-per-pixel).
    z_min, z_max = 0, 250  # The min and max elevation values in the data.
    is_y_flipped = False  # Whether to flip the heightmap in the Y dimension.

    # Create a paraboloid terrain.
    a, b = 1, 1
    a_sq_inv, b_sq_inv = 1.0 / (a * a), 1.0 / (b * b)
    hmap = {}
    for x in range(w):
        d_inner = {}
        x_comp = (x * x) * a_sq_inv
        for y in range(h):
            raw_val = x_comp + (y * y) * b_sq_inv
            d_inner[y] = min(z_max, max(z_min, raw_val * 0.0004))
        hmap[x] = d_inner

    # Import the terrain to BeamNG.
    Terrain_Importer.import_heightmap(
        beamng, hmap, w, h, scale, z_min, z_max, is_y_flipped
    )

    # Execute BeamNG until the user closes it.
    print("Completed.")
    while True:
        pass
    beamng.close()


if __name__ == "__main__":
    main()
