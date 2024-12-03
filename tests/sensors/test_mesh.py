from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Mesh


def test_mesh(beamng: BeamNGpy, steps: int = 3):
    with beamng as bng:
        # Create a vehicle.
        vehicle = Vehicle("ego_vehicle", model="etki", licence="PYTHON", color="Red")
        # Create a scenario.
        scenario = Scenario("tech_ground", "mesh_test2", description="Mesh analysis")
        # Add the vehicle to the scenario.
        scenario.add_vehicle(vehicle)
        scenario.make(bng)
        # Set simulator to 60hz temporal resolution
        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        print("Mesh test start.")

        # Get the mesh data for the vehicle.
        mesh = Mesh("mesh1", bng, vehicle)

        nodes = mesh.get_node_positions()
        print("Current node positions from simulator:")
        print(nodes)

        # vehicle.ai.set_mode('traffic')
        for _ in range(steps):
            sleep(5)
            data = mesh.poll()
            print(data)
            print("data time: ", data["time"])
            mesh.mesh_plot()
            mesh.mass_distribution_plot(data["nodes"])
            mesh.force_distribution_plot(data["nodes"])
            mesh.force_direction_plot(data["nodes"])
            mesh.velocity_distribution_plot(data["nodes"])
            mesh.velocity_direction_plot(data["nodes"])


if __name__ == "__main__":
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    test_mesh(bng, steps=100)
