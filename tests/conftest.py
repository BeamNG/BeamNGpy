from __future__ import annotations

import os
from typing import Iterator

import matplotlib
import pytest

from beamngpy import BeamNGpy
from beamngpy.logging import BNGDisconnectedError

GFX_BACKENDS = ("d3d12", "dx11", "vk")


def pytest_addoption(parser: pytest.Parser) -> None:
    group = parser.getgroup("beamng")
    group.addoption(
        "--gfx",
        action="store",
        default=None,
        choices=list(GFX_BACKENDS),
        help="Force one graphics backend. Overrides the GFX environment variable.",
    )
    group.addoption(
        "--gfx-loop",
        action="store_true",
        default=False,
        help="Run the full suite once per graphics backend (d3d12, dx11, vk). "
        "Relaunches BeamNG between backends.",
    )


def pytest_generate_tests(metafunc: pytest.Metafunc) -> None:
    if not metafunc.config.getoption("--gfx-loop"):
        return
    # pytest errors if the parametrized fixture is not in this test's tree
    # (offline tests such as test_minimal do not use BeamNG).
    if "gfx_backend" not in metafunc.fixturenames:
        return
    metafunc.parametrize(
        "gfx_backend",
        GFX_BACKENDS,
        ids=list(GFX_BACKENDS),
        scope="session",
        indirect=True,
    )


def pytest_report_header(config: pytest.Config) -> list[str]:
    headless = os.environ.get("HEADLESS", "0")
    if config.getoption("--gfx-loop"):
        return [f"BeamNG gfx: looping {', '.join(GFX_BACKENDS)}  HEADLESS={headless}"]
    gfx = config.getoption("--gfx") or os.environ.get("GFX") or "default"
    return [f"BeamNG gfx: {gfx}  HEADLESS={headless}"]


@pytest.fixture(autouse=True, scope="session")
def run_before_tests():
    matplotlib.use(
        "Agg"
    )  # do not show matplotlib plots in tests so they can be run automatically


@pytest.fixture(scope="session")
def gfx_backend(request: pytest.FixtureRequest) -> str | None:
    if hasattr(request, "param"):
        return request.param
    cli = request.config.getoption("--gfx")
    if cli:
        return cli
    return os.environ.get("GFX") or None


@pytest.fixture(scope="session")
def beamng(gfx_backend: str | None) -> Iterator[BeamNGpy]:
    headless = os.environ.get("HEADLESS", "0") == "1"
    nogpu = os.environ.get("NOGPU", "0") == "1"
    bng = BeamNGpy(
        "localhost",
        25252,
        quit_on_close=False,
        debug=False,
        headless=headless,
        nogpu=nogpu,
        gfx=gfx_backend,
        socket_timeout=240,
    )
    bng.open(None, "-no-tcom-debug")  # launch BeamNG if necessary
    bng.disconnect()  # to show no socket warning on next open()
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
