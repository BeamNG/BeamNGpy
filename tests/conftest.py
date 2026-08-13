from __future__ import annotations

from typing import Iterator
import os

import matplotlib
import pytest

from beamngpy import BeamNGpy
from beamngpy.logging import BNGDisconnectedError


@pytest.fixture(autouse=True, scope="session")
def run_before_tests():
    matplotlib.use(
        "Agg"
    )  # do not show matplotlib plots in tests so they can be run automatically


@pytest.fixture(scope="session")
def beamng() -> Iterator[BeamNGpy]:
    headless = os.environ.get("HEADLESS", "0") == "1"
    nogpu = os.environ.get("NOGPU", "0") == "1"
    bng = BeamNGpy("localhost", 25252, quit_on_close=False, debug=False, headless=headless, nogpu=nogpu)
    yield bng

    # Teardown after all tests have run
    bng.quit_on_close = True
    if bng.connection is None:
        try:
            bng.open(launch=False)
        except BNGDisconnectedError:
            pass
    bng.close()
