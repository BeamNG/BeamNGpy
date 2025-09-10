from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging

def main():
    set_up_simple_logging()

    beamng = BeamNGpy("localhost", 25252)
    beamng.open(launch=True)

    scenario = Scenario("hirochi_raceway", "ai_waypoints")

    vehicle = Vehicle("ego_vehicle", model="etk800", license="AI")

    scenario.add_vehicle(vehicle, pos=(-408.5, 260.2, 25.22), rot_quat=(-0.006664, -0.002505, -0.2799, 0.96))
    scenario.make(beamng)

    beamng.scenario.load(scenario)
    beamng.scenario.start()

    # beamng.traffic.spawn()

    print("Lapping Hirochi Raceway three times...")
    vehicle.ai.drive_using_waypoints(
        ['hr_start', 'quickrace_wp1', 'quickrace_wp2', 'quickrace_wp3', 'quickrace_wp4', 'quickrace_wp11', 'hr_start'],
        drive_in_lane=False,
        avoid_cars=True,
        no_of_laps=3,
        route_speed=100.0/3.6
    )

    input("Press Enter to exit...")
    beamng.disconnect()


if __name__ == "__main__":
    main()
