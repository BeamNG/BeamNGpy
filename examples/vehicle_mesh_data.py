from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Mesh


def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    bng.open(launch=True)
    # Create a vehicle.
    vehicle = Vehicle("ego_vehicle", model="etki", licence="PYTHON", color="Red")
    # Create a scenario.
    scenario = Scenario("industrial", "mesh_test", description="Mesh analysis")
    # Add the vehicle to the scenario.
    scenario.add_vehicle(vehicle, pos=(408.36, -336.56, 35.54), rot_quat=(0, 0, 0.99, -0.17))
    scenario.make(bng)
    # Set simulator to 60hz temporal resolution
    bng.settings.set_deterministic(60)
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # Get the mesh data for the vehicle.
    mesh = Mesh("mesh1", bng, vehicle)

    print("Driving around, polling the mesh data every 5 seconds...")
    vehicle.ai.set_mode("traffic")
    for _ in range(3):
        sleep(5)
        data = mesh.poll()
        # Note: there may be other readings in indices > 0, depending on mesh sensor update rates.
        print("data time: ", data["time"])
        mesh.mesh_plot()
        mesh.mass_distribution_plot(data["nodes"])
        mesh.force_distribution_plot(data["nodes"])
        mesh.force_direction_plot(data["nodes"])
        mesh.velocity_distribution_plot(data["nodes"])
        mesh.velocity_direction_plot(data["nodes"])

    vehicle.ai.set_mode("disabled")
    bng.ui.show_hud()
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
