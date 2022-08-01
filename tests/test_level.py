import pytest
from beamngpy import BeamNGpy


@pytest.fixture
def beamng():
    beamng = BeamNGpy('localhost', 64256)
    return beamng


def test_get_levels(beamng):
    with beamng as bng:
        levels = bng.get_levels()
        assert len(levels) > 0
