from __future__ import annotations

import pytest

from beamngpy import BeamNGpy


@pytest.fixture
def beamng():
    beamng = BeamNGpy('localhost', 64256, quit_on_close=False)
    return beamng


def test_get_levels(beamng: BeamNGpy):
    with beamng as bng:
        levels = bng.scenario.get_levels()
        assert len(levels) > 0
