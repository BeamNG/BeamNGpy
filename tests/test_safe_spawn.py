from beamngpy import BeamNGpy, Scenario, Vehicle, ScenarioObject


# DO NOT USE SMALLGRID
test_cases = [
    { # z+0 and safe_spawn = True -> vehicle is positioned at ground
        'map': "tech_ground",
        'vehicle': "etk800",
        'pos': (0, 0, 0),
        'safe_spawn': True,
        'expected_z_range': (0.0, 0.22),
    },
    { # z+-1 and safe_spawn = True -> vehicle is positioned at ground
        'map': "tech_ground",
        'vehicle': "etk800",
        'pos': (0, 0, -1),
        'safe_spawn': True,
        'expected_z_range': (0.0, 0.22),
    },
    { # z+3 and safe_spawn = True -> vehicle is positioned at ground
        'map': "tech_ground",
        'vehicle': "etk800",
        'pos': (0, 0, 3),
        'safe_spawn': True,
        'expected_z_range': (0.0, 0.22),
    },
    { # z+50 and safe_spawn = True -> Too high for safe_spawn anyway, vehicle falls from above ground
        'map': "tech_ground",
        'vehicle': "etk800",
        'pos': (0, 0, 50),
        'safe_spawn': True,
        'expected_z_range': (45, 55),
    },
    { # z+0 and safe_spawn = False -> vehicle spawns through ground
        'map': "tech_ground",
        'vehicle': "etk800",
        'pos': (0, 0, 0),
        'safe_spawn': False,
        'expected_z_range': (0.0, 0.1),
    },
    { # z+-1 and safe_spawn = False -> vehicle falls under ground
        'map': "tech_ground",
        'vehicle': "etk800",
        'pos': (0, 0, -1),
        'safe_spawn': False,
        'expected_z_range': (-1, -0.8),
    },
    { # z+3 and safe_spawn = False -> vehicle falls from above ground
        'map': "tech_ground",
        'vehicle': "etk800",
        'pos': (0, 0, 3),
        'safe_spawn': False,
        'expected_z_range': (2.0, 3.0),
    },
    { # z+50 and safe_spawn = False -> vehicle falls from above ground
        'map': "tech_ground",
        'vehicle': "etk800",
        'pos': (0, 0, 50),
        'safe_spawn': False,
        'expected_z_range': (45, 55),
    },
    
    { # z+0 and safe_spawn = True -> vehicle is positioned at ground
        'map': "small_island",
        'vehicle': "etk800",
        'pos': (-167, -207, 69.0),
        'safe_spawn': True,
        'expected_z_range': (68.9, 69.0),
    },
    { # z+-1 and safe_spawn = True -> vehicle is positioned at ground
        'map': "small_island",
        'vehicle': "etk800",
        'pos': (-167, -207, 68.0),
        'safe_spawn': True,
        'expected_z_range': (68.9, 69.0),
    },
    { # z+3 and safe_spawn = True -> vehicle is positioned at ground
        'map': "small_island",
        'vehicle': "etk800",
        'pos': (-167, -207, 72.0),
        'safe_spawn': True,
        'expected_z_range': (68.9, 69.0),
    },
    { # z+50 and safe_spawn = True -> Too high for safe_spawn anyway, vehicle falls from above ground
        'map': "small_island",
        'vehicle': "etk800",
        'pos': (-167, -207, 120.0),
        'safe_spawn': True,
        'expected_z_range': (115, 125),
    },
    { # z+0 and safe_spawn = False -> vehicle spawns through ground
        'map': "small_island",
        'vehicle': "etk800",
        'pos': (-167, -207, 68.0),
        'safe_spawn': False,
        'expected_z_range': (68.0, 68.3),
    },
    { # z+-1 and safe_spawn = False -> vehicle falls under ground
        'map': "small_island",
        'vehicle': "etk800",
        'pos': (-167, -207, 67.0),
        'safe_spawn': False,
        'expected_z_range': (60.0, 67.0),
    },
    { # z+3 and safe_spawn = False -> vehicle falls from above ground
        'map': "small_island",
        'vehicle': "etk800",
        'pos': (-167, -207, 71.0),
        'safe_spawn': False,
        'expected_z_range': (69.0, 71.0),
    },
    { # z+50 and safe_spawn = False -> vehicle falls from above ground
        'map': "small_island",
        'vehicle': "etk800",
        'pos': (-167, -207, 120.0),
        'safe_spawn': False,
        'expected_z_range': (115, 125),
    },
]


def test_safe_spawn_vehicle_spawn(beamng: BeamNGpy):
    """
    Test that the safe_spawn parameter works correctly :
    
    - for a spawnpoint where ground level is 0 m (e.g. smallgrid)
    - for a spawnpoint where ground level is above 0 m (e.g. 30 m)
    - for a spawnpoint where ground level is below 0 m (e.g. -30 m)
    
    - safe_spawn=True
        - for pos.z high value above ground -> vehicle is positioned at ground
        - for pos.z low value above ground -> vehicle is positioned at ground
        - for pos.z value at ground -> vehicle is positioned at ground
        - for pos.z value below ground -> vehicle is positioned at ground
        - all cases: same z position expected

    - safe_spawn=False
        - for pos.z high value above ground -> vehicle is positioned at specified position
        - for pos.z low value above ground -> vehicle is positioned at specified position
        - for pos.z value at ground -> vehicle is positioned at specified position
        - for pos.z value below ground -> vehicle is positioned at specified position
    """
    
    with beamng as bng:
        for i, test_case in enumerate(test_cases):
                
            name = f"test_spawnvehicle_{test_case['map']}_{test_case['vehicle']}_{test_case['safe_spawn']}_z{test_case['pos'][2]}"
            
            vehicle = Vehicle(f"test_car_{i}", model=test_case['vehicle'])
            scenario = Scenario(test_case['map'], name)
            scenario.make(bng)
            bng.control.pause()
            bng.scenario.load(scenario)
            bng.scenario.start()
            scenario.add_vehicle(vehicle, pos=test_case['pos'], safe_spawn=test_case['safe_spawn'])
            
            bng.control.step(10)
            vehicle.sensors.poll()

            assert test_case['expected_z_range'][0] <= vehicle.sensors["state"]["pos"][2] <= test_case['expected_z_range'][1], f"Test case {name} failed: expected z range {test_case['expected_z_range']}, got {vehicle.sensors['state']['pos'][2]}"
            

def test_safe_spawn_vehicle_prefab(beamng: BeamNGpy):
    """
    Test that the safe_spawn parameter works correctly when the vehicle is added
    to a scenario before it is loaded (prefab / spawnAutoplace path).
    """
    with beamng as bng:
        for i, test_case in enumerate(test_cases):
                
            name = f"test_prefab_spawn_{test_case['map']}_{test_case['vehicle']}_{test_case['safe_spawn']}_z{test_case['pos'][2]}"
            
            vehicle = Vehicle(f"test_car_{i}", model=test_case['vehicle'])
            scenario = Scenario(test_case['map'], name)
            scenario.add_vehicle(vehicle, pos=test_case['pos'], safe_spawn=test_case['safe_spawn'])
            scenario.make(bng)
            bng.control.pause()
            bng.scenario.load(scenario)
            bng.scenario.start()
            
            bng.control.step(10)
            vehicle.sensors.poll()
            
            assert test_case['expected_z_range'][0] <= vehicle.sensors["state"]["pos"][2] <= test_case['expected_z_range'][1], f"Test case {name} failed: expected z range {test_case['expected_z_range']}, got {vehicle.sensors['state']['pos'][2]}"

def test_safe_teleport_vehicle(beamng: BeamNGpy):
    """
    Test that the safe_spawn parameter works correctly for a teleported vehicle.
    Note : works also with cling instead of safe_spawn (alias for safe_spawn for backward compatibility)
    """
    with beamng as bng:
        vehicle = Vehicle("test_car", model="etk800")
        scenario = Scenario("tech_ground", "test_teleport_vehicle")
        scenario.make(bng)
        bng.control.pause()
        bng.scenario.load(scenario)
        bng.scenario.start()
        
        # Init vehicle at ground level
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), safe_spawn=True)
        bng.control.step(10)
        vehicle.sensors.poll()
        assert 0.0 <= vehicle.sensors["state"]["pos"][2] <= 0.22
        
        # Teleport vehicle to ground level with safe_spawn=True
        vehicle.teleport(pos=(5, 0, 0), safe_spawn=True)
        bng.control.step(10)
        vehicle.sensors.poll()
        # 0.2 m tolerance for the x and y coordinates
        assert 4.8 <= vehicle.sensors["state"]["pos"][0] <= 5.2
        assert -0.2 <= vehicle.sensors["state"]["pos"][1] <= 0.2
        assert 0.0 <= vehicle.sensors["state"]["pos"][2] <= 0.22
        
        # Teleport vehicle in the air with safe_spawn=True
        vehicle.teleport(pos=(5, 5, 5), safe_spawn=True)
        bng.control.step(10)
        vehicle.sensors.poll()
        # 0.2 m tolerance for the x and y coordinates for this test
        assert 4.8 <= vehicle.sensors["state"]["pos"][0] <= 5.2
        assert 4.8 <= vehicle.sensors["state"]["pos"][1] <= 5.2
        assert 0.0 <= vehicle.sensors["state"]["pos"][2] <= 0.22
        
        # Teleport vehicle in the air with safe_spawn=False
        vehicle.teleport(pos=(0, 5, 5), safe_spawn=False)
        bng.control.step(10)
        vehicle.sensors.poll()
        assert -0.2 <= vehicle.sensors["state"]["pos"][0] <= 0.2
        assert 4.8 <= vehicle.sensors["state"]["pos"][1] <= 5.2
        assert 4 <= vehicle.sensors["state"]["pos"][2] <= 6
        
def test_safe_teleport_scenario_object(beamng: BeamNGpy):
    """
    Test that the cling and offset parameters work correctly for teleport_object.

    Uses tech_ground (flat map, ground at z=0) so cling always snaps to z=0.
    """

    with beamng as bng:
        scenario = Scenario("tech_ground", "test_teleport_scenario_object")

        waypoint = ScenarioObject(
            oid="test_wp",
            name="test_wp",
            otype="BeamNGWaypoint",
            pos=(0, 0, 0),
            scale=(1, 1, 1),
            rot_quat=(0, 0, 0, 1),
        )
        scenario.add_object(waypoint)
        scenario.make(bng)
        bng.scenario.load(scenario)
        bng.scenario.start()

        def get_pos():
            objects = bng.scenario.find_objects_class("BeamNGWaypoint")
            for obj in objects:
                if obj.name == "test_wp":
                    return obj.pos
            raise AssertionError("test_wp not found in scene")

        # Verify initial position at (0, 0, 0)
        pos = get_pos()
        assert abs(pos[0]) <= 0.2
        assert abs(pos[1]) <= 0.2
        assert abs(pos[2]) <= 0.2

        # Teleport without cling: object should land exactly at (0, 5, 5)
        bng.scenario.teleport_object(waypoint, (0, 5, 5))
        pos = get_pos()
        assert abs(pos[0] - 0) <= 0.05
        assert abs(pos[1] - 5) <= 0.05
        assert abs(pos[2] - 5) <= 0.05

        # Teleport with cling (no offset): z should snap to ground (0), x/y preserved
        bng.scenario.teleport_object(waypoint, (5, 0, 5), cling=True)
        pos = get_pos()
        assert abs(pos[0] - 5) <= 0.2
        assert abs(pos[1] - 0) <= 0.2
        assert abs(pos[2] - 0) <= 0.2

        # Teleport with cling and offset=1.0: z should snap to ground + 1.0 = 1.0
        bng.scenario.teleport_object(waypoint, (5, 5, 5), cling=True, offset=1.0)
        pos = get_pos()
        assert abs(pos[0] - 5) <= 0.2
        assert abs(pos[1] - 5) <= 0.2
        assert abs(pos[2] - 1) <= 0.2


if __name__ == "__main__":
    beamng = BeamNGpy("localhost", 25252)
    beamng.open(launch=True)
    test_safe_spawn_vehicle_spawn(beamng)
    test_safe_spawn_vehicle_prefab(beamng)
    test_safe_teleport_vehicle(beamng)
    test_safe_teleport_scenario_object(beamng)
    beamng.close()