from __future__ import annotations

import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.logging import BNGValueError


def test_multi_vehicle(beamng: BeamNGpy):
    """
    Test that a second client can connect to a running instance, check for
    active vehicles, connect to one, and control it
    """
    with beamng as a_client:
        scenario = Scenario("tech_ground", "multi_vehicle")
        first = Vehicle("first", model="etk800")
        scenario.add_vehicle(first, pos=(2, 2, 0))
        second = Vehicle("second", model="etki")
        scenario.add_vehicle(second, pos=(-2, -2, 0))
        scenario.make(a_client)

        a_client.scenario.load(scenario)
        a_client.scenario.start()

        b_client = BeamNGpy("localhost", 25252)
        #  Do not launch new process
        b_client.open(launch=False)
        vehicles = b_client.vehicles.get_current(include_config=False)
        assert "second" in vehicles
        vehicle = vehicles["second"]
        vehicle.connect(b_client)
        assert vehicle.is_connected()

        a_veh = second
        b_veh = vehicle

        b_veh.control(throttle=1.0)

        for _ in range(8):
            # Verify position updating in both clients
            a_veh.sensors.poll()
            b_veh.sensors.poll()
            a_ref = a_veh.sensors["state"]["pos"]
            b_ref = b_veh.sensors["state"]["pos"]
            b_client.control.step(100)
            a_veh.sensors.poll()
            b_veh.sensors.poll()
            a_new = a_veh.sensors["state"]["pos"]
            b_new = b_veh.sensors["state"]["pos"]

            assert a_ref[0] != a_new[0] or a_ref[1] != a_new[1]
            assert b_ref[0] != b_new[0] or b_ref[1] != b_new[1]


def test_multi_scenario(beamng: BeamNGpy):
    """
    Test that a second client can connect to a running instance and ask
    information about the loaded scenario.
    """
    with beamng as a_client:
        a_client.control.return_to_main_menu()  # if a scenario was running previously
        scenario = Scenario("gridmap_v2", "multi_scenario")
        vehicle = Vehicle("vehicle", model="etk800")
        scenario.add_vehicle(vehicle, pos=(0, 0, 100))
        scenario.make(a_client)

        b_client = BeamNGpy("localhost", 25252)
        b_client.open(launch=False)

        with pytest.raises(BNGValueError):
            running = b_client.scenario.get_current()

        a_client.scenario.load(scenario)
        a_client.scenario.start()

        running = b_client.scenario.get_current()
        assert str(running.level) == scenario.level
        assert running.name == scenario.name
