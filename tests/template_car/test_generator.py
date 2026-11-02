"""TemplateCarGenerator facade, settings/path resolution, worker entry point and shortcuts."""

from __future__ import annotations

import json
import os
import platform
import sys
from pathlib import Path

import pytest

from beamngpy.tools.template_car import (
    OptimizationEnabled,
    OptimizationVariables,
    Optimization,
    Settings,
    TemplateCarGenerator,
    TemplateVehicle,
)
from beamngpy.tools.template_car import core, optimization, shortcut, utils, worker

from .helpers import read_body_nodes, vehicle_files_dir

FALLBACK_INSTALL_PATH = "C:\\BeamNG" if platform.system() == "Windows" else "/BeamNG"


def _write_settings(tmp_path: Path, **settings) -> TemplateCarGenerator:
    file = tmp_path / "settings.json"
    file.write_text(json.dumps(settings), encoding="utf-8")
    return TemplateCarGenerator(str(file))


@pytest.fixture
def data_path(tmp_path, monkeypatch) -> Path:
    path = tmp_path / "data"
    monkeypatch.setattr(utils, "default_beamng_data_path", lambda: str(path))
    return path


def test_default_settings(data_path):
    generator = TemplateCarGenerator()
    assert generator.settings_file == str(data_path / "TemplateCarGenerator" / "settings.json")
    assert generator.get_settings() == Settings()
    assert generator.user_folder == str(data_path / "BeamNG.tech")
    assert generator.vehicles_folder == str(data_path / "TemplateCarGenerator" / "vehicles")


def test_default_beamng_data_path_windows(monkeypatch, tmp_path):
    monkeypatch.setattr(utils.platform, "system", lambda: "Windows")
    monkeypatch.setenv("LOCALAPPDATA", str(tmp_path))
    assert utils.default_beamng_data_path() == os.path.join(str(tmp_path), "BeamNG")


def test_default_beamng_data_path_linux(monkeypatch):
    monkeypatch.setattr(utils.platform, "system", lambda: "Linux")
    assert utils.default_beamng_data_path() == os.path.join(os.path.expanduser("~/.local/share"), "BeamNG")


def test_relative_settings_file_is_made_absolute(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert TemplateCarGenerator("settings.json").settings_file == str(tmp_path / "settings.json")


def test_settings_from_file(generator, user_folder, tmp_path):
    assert generator.get_settings() == Settings(
        user_folder=str(user_folder),
        vehicles_folder=str(tmp_path / "vehicles"),
        beamng_install_path=str(tmp_path / "beamng"),
    )
    assert generator.user_folder == str(user_folder)
    assert generator.vehicles_folder == str(tmp_path / "vehicles")
    assert generator.install_path == str(tmp_path / "beamng")


class TestInstallPath:
    @pytest.fixture(autouse=True)
    def no_bng_home(self, monkeypatch):
        monkeypatch.delenv("BNG_HOME", raising=False)

    def _generator(self, tmp_path, user_folder, install_path=""):
        return _write_settings(tmp_path, user_folder=str(user_folder), beamng_install_path=install_path)

    def test_setting_wins(self, tmp_path, user_folder, monkeypatch):
        monkeypatch.setenv("BNG_HOME", str(tmp_path))
        assert self._generator(tmp_path, user_folder, "D:\\custom").install_path == "D:\\custom"

    def test_bng_home(self, tmp_path, user_folder, monkeypatch):
        home = tmp_path / "home"
        home.mkdir()
        monkeypatch.setenv("BNG_HOME", str(home))
        assert self._generator(tmp_path, user_folder).install_path == str(home)

    def test_missing_bng_home_is_ignored(self, tmp_path, user_folder, monkeypatch):
        monkeypatch.setenv("BNG_HOME", str(tmp_path / "missing"))
        assert self._generator(tmp_path, user_folder).install_path == FALLBACK_INSTALL_PATH

    @pytest.mark.parametrize("ini_name", ["BeamNG.tech.ini", "BeamNG.drive.ini"])
    def test_ini_file_next_to_user_folder(self, tmp_path, user_folder, ini_name):
        (tmp_path / ini_name).write_text("someSetting = 1\ninstallPath = E:\\Games\\BeamNG\n", encoding="utf-8")
        assert self._generator(tmp_path, user_folder).install_path == "E:\\Games\\BeamNG"

    def test_tech_ini_is_preferred(self, tmp_path, user_folder):
        (tmp_path / "BeamNG.tech.ini").write_text("installPath = tech\n", encoding="utf-8")
        (tmp_path / "BeamNG.drive.ini").write_text("installPath = drive\n", encoding="utf-8")
        assert self._generator(tmp_path, user_folder).install_path == "tech"

    def test_fallback(self, tmp_path, user_folder):
        assert self._generator(tmp_path, user_folder).install_path == FALLBACK_INSTALL_PATH


def test_vehicle_list_and_get(generator):
    folder = generator.vehicles_folder
    vehicles = {"b_car": TemplateVehicle(name="B"), "a_car": TemplateVehicle(name="A")}
    for vehicle_id, vehicle in vehicles.items():
        utils.save_vehicle_file(folder, vehicle_id, vehicle)
    Path(folder, "notes.txt").write_text("", encoding="utf-8")

    assert generator.get_vehicle_list() == ["a_car", "b_car"]
    for vehicle_id, vehicle in vehicles.items():
        assert generator.get_vehicle(vehicle_id) == vehicle


def test_get_missing_vehicle(generator):
    with pytest.raises(FileNotFoundError):
        generator.get_vehicle("missing")


def test_generate_without_optimization(generator, user_folder, monkeypatch):
    def no_simulator(*args, **kwargs):
        raise AssertionError("optimize() must not run without enabled targets")

    monkeypatch.setattr(optimization, "optimize", no_simulator)
    vehicle = TemplateVehicle(name="No targets", optimization=Optimization(variables=OptimizationVariables(mass_factor=1.2)))
    generator.generate("no_targets", vehicle)

    assert utils.mod_exists(str(user_folder), "no_targets")
    nodes = read_body_nodes(vehicle_files_dir(user_folder, "no_targets"), "sedan")
    assert nodes["b2"][3] == pytest.approx(24.0)
    assert nodes["rf2"][3] == pytest.approx(12.0)


def test_generate_passes_settings(generator, user_folder, tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(core, "generate_optimal_car", lambda *args: calls.append(args))
    vehicle = TemplateVehicle(name="Wiring")
    generator.generate("wiring", vehicle)
    assert calls == [("wiring", vehicle, str(user_folder), str(tmp_path / "beamng"))]


def test_play(generator, user_folder, tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(core, "determine_binary", lambda home: calls.append(("binary", home)) or "beamng.exe")
    monkeypatch.setattr(core.subprocess, "run", lambda args, **kwargs: calls.append((args, kwargs)))
    generator.play("my_car", "west_coast_usa")
    assert calls == [
        ("binary", tmp_path / "beamng"),
        (
            ["beamng.exe", "-level", "west_coast_usa", "-vehicle", "my_car", "-userpath", str(user_folder)],
            {"check": True},
        ),
    ]


def _run_worker(monkeypatch, vehicle_id: str, settings_file: Path) -> None:
    monkeypatch.setattr(sys, "argv", ["worker", vehicle_id, str(settings_file)])
    worker.main()


def test_worker_generates_vehicle(generator, settings_file, user_folder, monkeypatch, capsys):
    utils.save_vehicle_file(generator.vehicles_folder, "worker_car", TemplateVehicle(name="Worker"))
    _run_worker(monkeypatch, "worker_car", settings_file)
    assert "--- ERROR ---" not in capsys.readouterr().out
    assert utils.mod_exists(str(user_folder), "worker_car")


def test_worker_reports_invalid_id(generator, settings_file, monkeypatch, capsys):
    utils.save_vehicle_file(generator.vehicles_folder, "Bad-Id", TemplateVehicle(name="Bad"))
    _run_worker(monkeypatch, "Bad-Id", settings_file)
    out = capsys.readouterr().out
    assert out.startswith(worker.error_string("Invalid ID: Bad-Id."))


def test_worker_reports_optimization_failure(generator, settings_file, monkeypatch, capsys):
    def fail(*args, **kwargs):
        raise ValueError("Optimization failed: boom")

    monkeypatch.setattr(optimization, "optimize", fail)
    vehicle = TemplateVehicle(name="Fails", optimization=Optimization(enabled=OptimizationEnabled(mass=True)))
    utils.save_vehicle_file(generator.vehicles_folder, "fails", vehicle)
    _run_worker(monkeypatch, "fails", settings_file)
    assert capsys.readouterr().out == (
        "Starting optimization using BeamNG. Please wait...\n"
        + worker.error_string("Optimization failed: boom")
        + "\n"
    )


def test_error_string():
    assert worker.error_string("text") == "\n--- ERROR ---\ntext"


def test_linux_shortcut(tmp_path):
    file = tmp_path / "BeamNG Template Car Generator.desktop"
    shortcut.create_shortcut_linux(str(file), "/work", "BeamNG.tech", "Launch it")
    assert file.read_text().splitlines() == [
        "[Desktop Entry]",
        "Name=BeamNG Template Car Generator",
        "Exec=python3 -m beamngpy.tools.template_car",
        "Type=Application",
        "Terminal=false",
        "Comment=Launch it",
        "Path=/work",
        "Icon=BeamNG.tech",
    ]
    if platform.system() != "Windows":
        assert file.stat().st_mode & 0o777 == 0o755


@pytest.mark.parametrize("vehicle_id", ["a", "car", "my_car_2", "x1_y2"])
def test_valid_ids(vehicle_id):
    utils.validate_id(vehicle_id)


def test_vehicle_file_helpers(tmp_path):
    folder = tmp_path / "nested" / "vehicles"
    assert utils.get_vehicle_path(str(folder), "car") == os.path.join(str(folder), "car.json")
    assert not utils.vehicle_exists(str(folder), "car")
    vehicle = TemplateVehicle(name="Saved")
    utils.save_vehicle_file(str(folder), "car", vehicle)
    assert utils.vehicle_exists(str(folder), "car")
    assert TemplateVehicle.model_validate(utils.read_json(utils.get_vehicle_path(str(folder), "car"))) == vehicle


def test_write_settings_creates_parent(tmp_path):
    file = tmp_path / "new" / "settings.json"
    settings = Settings(user_folder="u", vehicles_folder="v", beamng_install_path="i")
    utils.write_settings(str(file), settings)
    assert Settings.model_validate(utils.read_json(str(file))) == settings


def test_mod_exists(user_folder):
    assert not utils.mod_exists(str(user_folder), "car")
    (user_folder / "current" / "mods" / "unpacked" / "car").mkdir(parents=True)
    assert utils.mod_exists(str(user_folder), "car")


def test_ensure_directory(tmp_path):
    folder = tmp_path / "a" / "b"
    utils.ensure_directory(str(folder))
    utils.ensure_directory(str(folder))
    assert folder.is_dir()
