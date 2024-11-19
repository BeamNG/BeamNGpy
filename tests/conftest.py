from __future__ import annotations

import matplotlib
import pytest

from beamngpy import BeamNGpy


@pytest.fixture(autouse=True, scope="session")
def run_before_tests():
    matplotlib.use(
        "Agg"
    )  # do not show matplotlib plots in tests so they can be run automatically


@pytest.fixture(scope="session")
def beamng() -> BeamNGpy:
    return BeamNGpy("localhost", 25252, quit_on_close=False, debug=False)
