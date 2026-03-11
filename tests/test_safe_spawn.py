from beamngpy import BeamNGpy, Scenario, Vehicle


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
            scenario.add_vehicle(vehicle, pos=test_case['pos'], cling=test_case['safe_spawn'])
            
            bng.control.step(10)
            vehicle.sensors.poll()

            assert test_case['expected_z_range'][0] <= vehicle.sensors["state"]["pos"][2] <= test_case['expected_z_range'][1], f"Test case {name} failed: expected z range {test_case['expected_z_range']}, got {vehicle.sensors['state']['pos'][2]}"
            

def test_spawn_autoplace_vehicle_spawn(beamng: BeamNGpy):
    """
    Same test, but with vehicle in prefab, using spawn_autoplace instead of safe_spawn parameter
    """
    with beamng as bng:
        for i, test_case in enumerate(test_cases):
                
            name = f"test_spawn_autoplace_vehicle_{test_case['map']}_{test_case['vehicle']}_{test_case['safe_spawn']}_z{test_case['pos'][2]}"
            
            vehicle = Vehicle(f"test_car_{i}", model=test_case['vehicle'], spawn_autoplace=test_case['safe_spawn'])
            scenario = Scenario(test_case['map'], name)
            scenario.add_vehicle(vehicle, pos=test_case['pos'])
            scenario.make(bng)
            bng.control.pause()
            bng.scenario.load(scenario)
            bng.scenario.start()
            
            bng.control.step(10)
            vehicle.sensors.poll()
            
            assert test_case['expected_z_range'][0] <= vehicle.sensors["state"]["pos"][2] <= test_case['expected_z_range'][1], f"Test case {name} failed: expected z range {test_case['expected_z_range']}, got {vehicle.sensors['state']['pos'][2]}"

if __name__ == "__main__":
    beamng = BeamNGpy("localhost", 25252)
    beamng.open(launch=True)
    test_safe_spawn_vehicle_spawn(beamng)
    test_spawn_autoplace_vehicle_spawn(beamng)
    beamng.close()