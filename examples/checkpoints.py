from beamngpy import BeamNGpy, Scenario, Vehicle

if __name__ == "__main__":
    with BeamNGpy("localhost", 25252) as bng:
        scenario = Scenario("tech_ground", "waypoint demo")
        vehicle = Vehicle("test_car", model="etk800", rot_quat=(0, 0, 0, 1))
        scenario.add_vehicle(vehicle)

        positions = [(0, -10, 0), (0, -20, 0), (0, -30, 0)]
        scales = [(1.0, 1.0, 1.0)] * 3
        scenario.add_checkpoints(positions, scales)
        scenario.make(bng)

        bng.scenario.load(scenario)
        bng.scenario.start()
        input("Press Enter to exit...")
