import pytest
from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.beamngcommon import BNGValueError


@pytest.fixture
def beamng():
    beamng = BeamNGpy('localhost', 64256)
    return beamng


def test_multi_vehicle(beamng):
    """
    Test that a second client can connect to a running instance, check for
    active vehicles, connect to one, and control it
    """
    with beamng as a_client:
        scenario = Scenario('smallgrid', 'multi_vehicle')
        first = Vehicle('first', model='etk800')
        scenario.add_vehicle(first, pos=(2, 2, 0))
        second = Vehicle('second', model='etki')
        scenario.add_vehicle(second, pos=(-2, -2, 0))
        scenario.make(a_client)

        a_client.load_scenario(scenario)
        a_client.start_scenario()

        b_client = BeamNGpy('localhost', 64256)
        #  Do not deploy mod zip or launch new process
        b_client.open(launch=False)
        vehicles = b_client.get_current_vehicles()
        assert 'second' in vehicles
        vehicle = vehicles['second']
        vehicle.connect(b_client)
        assert vehicle.is_connected()

        a_veh = second
        b_veh = vehicle

        b_veh.control(throttle=1.0)

        for _ in range(8):
            # Verify position updating in both clients
            a_veh.poll_sensors()
            b_veh.poll_sensors()
            a_ref = a_veh.sensors['state'].data['pos']
            b_ref = b_veh.sensors['state'].data['pos']
            b_client.step(100)
            a_veh.poll_sensors()
            b_veh.poll_sensors()
            a_new = a_veh.sensors['state'].data['pos']
            b_new = b_veh.sensors['state'].data['pos']

            assert a_ref[0] != a_new[0] or a_ref[1] != a_new[1]
            assert b_ref[0] != b_new[0] or b_ref[1] != b_new[1]


def test_multi_scenario(beamng):
    """
    Test that a second client can connect to a running instance and ask
    information about the loaded scenario.
    """
    with beamng as a_client:
        scenario = Scenario('gridmap_v2', 'multi_scenario')
        vehicle = Vehicle('vehicle', model='etk800')
        scenario.add_vehicle(vehicle, pos=(0, 0, 100))
        scenario.make(a_client)

        b_client = BeamNGpy('localhost', 64256)
        b_client.open(launch=False)

        with pytest.raises(BNGValueError):
            running = b_client.get_current_scenario()

        a_client.load_scenario(scenario)
        a_client.start_scenario()

        running = b_client.get_current_scenario()
        assert running.level == scenario.level
        assert running.name == scenario.name
