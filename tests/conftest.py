from __future__ import annotations

import os
from typing import Iterator

import matplotlib
import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle
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
    gfx = os.environ.get("GFX", None)
    bng = BeamNGpy(
        "localhost",
        25252,
        quit_on_close=False,
        debug=False,
        headless=headless,
        nogpu=nogpu,
        gfx=gfx,
        socket_timeout=300,  # for warmup only
    )
    bng.open(None, "-no-tcom-debug")  # launch BeamNG if necessary
    warmup_scenario(bng)  # warmup scenario with long timeout
    bng.disconnect()  # to show no socket warning on next open()
    bng.socket_timeout = 120  # for subsequent tests
    yield bng

    # Teardown: close BeamNG instance only if we launched it
    if bng.process is not None:
        bng.quit_on_close = True
        if bng.connection is None:
            try:
                bng.open(launch=False)
            except BNGDisconnectedError:
                pass
        bng.close()


def warmup_scenario(beamng: BeamNGpy):
    scenario = Scenario("smallgrid", "test_scenario")
    vehicle = Vehicle("test_car", model="template_car")
    scenario.add_vehicle(vehicle, pos=(0, 0, 0))
    scenario.make(beamng)
    beamng.scenario.load(scenario)
    beamng.scenario.start()
    beamng.scenario.stop()
