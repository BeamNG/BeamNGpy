from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging


def main():
    setup_logging()

    bng = BeamNGpy('localhost', 64256)
    scenario = Scenario('smallgrid', 'research_test',
                        description='Random driving for research')

    vehicle = Vehicle('ego_vehicle')
    scenario.add_vehicle(vehicle)

    scenario.make(bng)


if __name__ == '__main__':
    main()
