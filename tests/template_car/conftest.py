from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
from typing import Iterator

import pytest

from beamngpy import BeamNGpy
from beamngpy.tools.template_car import TemplateCarGenerator
from beamngpy.tools.template_car import optimization


@pytest.fixture
def user_folder(tmp_path: Path) -> Path:
    folder = tmp_path / "user"
    folder.mkdir()
    return folder


@pytest.fixture
def settings_file(tmp_path: Path, user_folder: Path) -> Path:
    file = tmp_path / "settings.json"
    settings = {
        "user_folder": str(user_folder),
        "vehicles_folder": str(tmp_path / "vehicles"),
        "beamng_install_path": str(tmp_path / "beamng"),
    }
    file.write_text(json.dumps(settings), encoding="utf-8")
    return file


@pytest.fixture
def generator(settings_file: Path) -> TemplateCarGenerator:
    return TemplateCarGenerator(str(settings_file))


@pytest.fixture(scope="session")
def sim_user(beamng: BeamNGpy) -> str:
    """User folder of the shared simulator, where generated cars have to be written."""
    user_with_version = Path(beamng.user_with_version or "")
    assert user_with_version.name == "current", (
        f"the generator writes to '<user>/current', but the simulator uses '{user_with_version}'"
    )
    return beamng.user


class _SharedSimulatorBeamNG(optimization.BeamNG):
    """Optimizer session that leaves the shared simulator running, without its car."""

    def __init__(self, *args):
        super().__init__(*args)
        self.beamng.quit_on_close = False

    def __exit__(self, *exc_info):
        try:
            # The files are deleted next, and BeamNG would hot-reload the still spawned car from them and disable it.
            if self.v is not None:
                self.beamng.vehicles.despawn(self.v)
        finally:
            super().__exit__(*exc_info)


@pytest.fixture(scope="module")
def optimizer_env(beamng: BeamNGpy, sim_user: str) -> Iterator[SimpleNamespace]:
    """Let the optimizer attach to the shared simulator instead of quitting it when it is done."""
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(optimization, "BeamNG", _SharedSimulatorBeamNG)
        yield SimpleNamespace(user=sim_user, home=beamng.home)
