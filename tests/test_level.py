from __future__ import annotations

from beamngpy import BeamNGpy


def test_get_levels(beamng: BeamNGpy):
    with beamng as bng:
        levels = bng.scenario.get_levels()
        assert len(levels) > 0
