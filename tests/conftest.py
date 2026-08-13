from __future__ import annotations

import os
from typing import Iterator

import matplotlib
import pytest

from beamngpy import BeamNGpy
from beamngpy.beamng.process import kill_stale_beamng_processes
from beamngpy.logging import BNGDisconnectedError


@pytest.fixture(autouse=True, scope="session")
def run_before_tests():
    matplotlib.use(
        "Agg"
    )  # do not show matplotlib plots in tests so they can be run automatically
    # Clear leftover BeamNG processes from a previous crashed/hung suite.
    kill_stale_beamng_processes()


@pytest.fixture(scope="session")
def beamng() -> Iterator[BeamNGpy]:
    headless = os.environ.get("HEADLESS", "0") == "1"
    nogpu = os.environ.get("NOGPU", "0") == "1"
    gfx = os.environ.get("GFX", None)
    bng = BeamNGpy(
        "localhost",
        25252,
        quit_on_close=False,
        debug=False,
        headless=headless,
        nogpu=nogpu,
        gfx=gfx,
        socket_timeout=120,
    )
    yield bng

    # Teardown after all tests have run
    bng.quit_on_close = True
    if bng.connection is None:
        try:
            bng.open(launch=False)
        except BNGDisconnectedError:
            pass
    try:
        bng.close()
    finally:
        kill_stale_beamng_processes()
