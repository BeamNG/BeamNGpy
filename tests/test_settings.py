from __future__ import annotations

from beamngpy import BeamNGpy


def test_get_physics_frequency(beamng: BeamNGpy):
    with beamng as bng:
        expected = float(
            bng.queue_lua_command(
                "return be:getPhysicsFrequency()",
                response=True,
            )
        )

        assert bng.settings.get_physics_frequency() == expected
        assert bng.get_physics_frequency() == expected
