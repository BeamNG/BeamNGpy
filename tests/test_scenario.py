
from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.beamngcommon import BNGValueError

import pytest


@pytest.fixture
def beamng():
    beamng = BeamNGpy('localhost', 64256)
    return beamng


def test_new_scenario(beamng):
    with beamng as bng:
        scenario = Scenario('smallgrid', 'test_scenario')
        vehicle = Vehicle('test_car', model='etk800')
        scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot=(0, 0, 0))
        scenario.make(bng)
        bng.load_scenario(scenario)
        assert bng.get_scenario_name() == 'test_scenario'

    scenario.delete(beamng)

    with beamng as bng:
        with pytest.raises(BNGValueError):
            bng.load_scenario(scenario)


def test_find_scenario(beamng):
    with beamng as bng:
        scenario = Scenario('west_coast_usa', 'derby_asphalt')
        scenario.find(bng)
        assert scenario.get_info_path() is not None
        bng.load_scenario(scenario)
        assert bng.get_scenario_name() == 'Asphalt Derby Race'
