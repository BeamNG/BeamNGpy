from beamngpy import BeamNGpy, Vehicle, Scenario, ScenarioObject, setup_logging



def main():
    setup_logging()

    
    beamng = BeamNGpy('localhost', 64256)

    scenario = Scenario('smallgrid', 'test_quat')

    vehicle = Vehicle('ego_vehicle', model='etk800', color='Blue', licence="angle")
    scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot=(0, 0, 0))


    vehicle = Vehicle('ego_vehicle2', model='etk800', color='Green', license="quat")
    scenario.add_vehicle(vehicle, pos=(5, 0, 0), rot_quat=(-0.00333699025,-0.00218820246,-0.689169466,0.724589229))

    rb = ScenarioObject(oid='roadblock', 
                              name='sawhorse',
                              otype='BeamNGVehicle',
                              pos=(-10, -5, 0),
                              rot=(0,0,0),
                              scale=(1, 1, 1),
                              JBeam = 'sawhorse',
                              datablock="default_vehicle"
                              )
    scenario.add_object(rb)

    cn = ScenarioObject(oid='cones', 
                              name='cones',
                              otype='BeamNGVehicle',
                              pos=(0, -5, 0),
                              rot=None, 
                              rot_quat=(0,0,0,1),
                              scale=(1, 1, 1),
                              JBeam = 'cones',
                              datablock="default_vehicle"
                              )
    scenario.add_object(cn)

    scenario.make(beamng)

    bng = beamng.open(launch=True)
    try:
        bng.load_scenario(scenario)
        bng.start_scenario()

        input('Press Enter to spawn vehicle during sim with rot and rotquat')
        vehicle = Vehicle('ego_vehicle3', model='etk800', color='White')
        bng.spawn_vehicle(vehicle, (-10,0,0), (0,0,0))

        vehicle = Vehicle('ego_vehicle4', model='pickup') 
        pos = (-15,0,0)
        bng.spawn_vehicle(vehicle, pos, None, rot_quat=(0,0,0,1))

        input('press Enter to teleport last vehicle with angle')
        bng.teleport_vehicle(vehicle, pos, rot=(0,45,0))

        input('press Enter to teleport last vehicle with quaternion')
        bng.teleport_vehicle(vehicle, pos, rot_quat=(-0.00333699025,-0.00218820246,-0.689169466,0.724589229))

        input('press Enter to teleport roadblock with angle')
        bng.teleport_scenario_object(rb, (-10, 5, 0), rot=(-45,0,0))

        input('press Enter to teleport roadblock with quaternion')
        bng.teleport_scenario_object(rb, (-10, 5, 0), rot_quat=(-0.003337, -0.0021882, -0.6891695, 0.7245892))

        input('Press ENTER to exit')
    finally:
        bng.close()


if __name__ == '__main__':
    main()
