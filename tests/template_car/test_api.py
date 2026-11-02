"""REST API of the Template Car Generator UI."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest
from fastapi import HTTPException
from fastapi.testclient import TestClient

from beamngpy.tools.template_car import Settings, TemplateCarGenerator, TemplateVehicle
from beamngpy.tools.template_car import api, utils


def _body(vehicle: TemplateVehicle) -> dict:
    return vehicle.model_dump(mode="json")


@pytest.fixture
def opened(monkeypatch) -> list[str]:
    paths = []
    monkeypatch.setattr(api, "ui_open", paths.append)
    return paths


@pytest.fixture
def client(generator, opened, monkeypatch) -> TestClient:
    monkeypatch.setattr(api, "generator", generator)
    monkeypatch.setattr(api, "workers", {})
    monkeypatch.setattr(api.send2trash, "send2trash", os.remove)
    return TestClient(api.app)


def test_web_application(client):
    response = client.get("/")
    assert response.status_code == 200
    assert response.headers["content-type"].startswith("text/html")


def test_static_file(client):
    response = client.get("/static/style.css")
    assert response.status_code == 200
    assert response.content == (Path(utils.web_dir()) / "static" / "style.css").read_bytes()


def test_static_rejects_parent_paths(generator, monkeypatch):
    monkeypatch.setattr(api, "generator", generator)
    response = TestClient(api.app, raise_server_exceptions=False).get("/static/%2e%2e/index.html")
    assert response.status_code == 500


def test_vehicle_lifecycle(client, generator):
    assert client.get("/vehicles").json() == []

    vehicle = TemplateVehicle(name="My car")
    response = client.post("/vehicles/my_car", json=_body(vehicle))
    assert response.status_code == 201
    assert TemplateVehicle.model_validate(response.json()) == vehicle
    assert utils.vehicle_exists(generator.vehicles_folder, "my_car")
    assert client.post("/vehicles/my_car", json=_body(vehicle)).status_code == 409

    assert client.get("/vehicles").json() == [{"id": "my_car", "name": "My car", "is_generated": False}]
    assert TemplateVehicle.model_validate(client.get("/vehicles/my_car").json()) == vehicle

    renamed = TemplateVehicle(name="Renamed")
    assert client.put("/vehicles/my_car", json=_body(renamed)).status_code == 200
    assert client.get("/vehicles/my_car").json()["name"] == "Renamed"

    assert client.delete("/vehicles/my_car").status_code == 204
    assert not utils.vehicle_exists(generator.vehicles_folder, "my_car")
    assert client.get("/vehicles/my_car").status_code == 404
    assert client.put("/vehicles/my_car", json=_body(renamed)).status_code == 404
    assert client.delete("/vehicles/my_car").status_code == 404


@pytest.mark.parametrize(
    "body",
    [
        {},
        {"name": "Car", "structure": {"body_shape": "hovercraft"}},
        {"name": "Car", "parameters": {"wheelbase": 20}},
        {"name": "Car", "optimization": {"targets": {"mass": 10}}},
    ],
    ids=["missing-name", "bad-body-shape", "wheelbase-out-of-range", "mass-target-out-of-range"],
)
def test_invalid_vehicle_is_rejected(client, generator, body):
    assert client.post("/vehicles/bad", json=body).status_code == 422
    assert not utils.vehicle_exists(generator.vehicles_folder, "bad")


def test_list_skips_broken_files(client, generator):
    utils.save_vehicle_file(generator.vehicles_folder, "good", TemplateVehicle(name="Good"))
    Path(generator.vehicles_folder, "broken.json").write_text("{not json", encoding="utf-8")
    Path(generator.vehicles_folder, "invalid.json").write_text('{"name": 1}', encoding="utf-8")
    assert client.get("/vehicles").json() == [{"id": "good", "name": "Good", "is_generated": False}]


def test_settings(client, generator):
    assert Settings.model_validate(client.get("/settings").json()) == generator.get_settings()
    settings = Settings(user_folder="u", vehicles_folder="v", beamng_install_path="i")
    assert client.put("/settings", json=settings.model_dump()).status_code == 200
    assert generator.get_settings() == settings
    assert Settings.model_validate(client.get("/settings").json()) == settings


def test_schemas(client):
    assert client.get("/schemas").json() == {
        "TemplateVehicle": TemplateVehicle.model_json_schema(),
        "Settings": Settings.model_json_schema(),
    }


def test_generate(client, generator, user_folder):
    client.post("/vehicles/api_car", json=_body(TemplateVehicle(name="API car")))
    response = client.post("/vehicles/api_car/generate")
    assert response.status_code == 200
    assert "--- ERROR ---" not in response.text
    assert utils.mod_exists(str(user_folder), "api_car")
    assert client.get("/vehicles").json() == [{"id": "api_car", "name": "API car", "is_generated": True}]
    assert api.workers == {}


def test_generate_streams_errors(client):
    client.post("/vehicles/Bad-Id", json=_body(TemplateVehicle(name="Bad")))
    response = client.post("/vehicles/Bad-Id/generate")
    assert response.status_code == 200
    assert "\n--- ERROR ---\nInvalid ID: Bad-Id." in response.text


def test_generate_missing_vehicle(client):
    assert client.post("/vehicles/missing/generate").status_code == 404


def test_cancel_without_worker(client):
    response = client.post("/vehicles/idle/cancel")
    assert response.status_code == 200
    assert response.json() == {"message": "Generation of vehicle 'idle' cancelled"}


def test_cancel_kills_worker(client):
    process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"], start_new_session=True)
    try:
        api.workers["busy"] = process
        assert client.post("/vehicles/busy/cancel").status_code == 200
        assert process.poll() is not None
        assert "busy" not in api.workers
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_play(client, generator, monkeypatch):
    calls = []
    monkeypatch.setattr(generator, "play", lambda vehicle_id, level_id: calls.append((vehicle_id, level_id)))
    response = client.post("/vehicles/my_car/play/smallgrid")
    assert response.status_code == 200
    assert response.json()["message"] == "BeamNG launched with vehicle 'my_car' on level 'smallgrid'"
    assert calls == [("my_car", "smallgrid")]


def test_play_failure(client, generator, monkeypatch):
    def fail(vehicle_id, level_id):
        raise FileNotFoundError("no binary")

    monkeypatch.setattr(generator, "play", fail)
    response = client.post("/vehicles/my_car/play/smallgrid")
    assert response.status_code == 500
    assert response.json()["detail"] == "BeamNG launch failed: no binary"


def test_ui_open_vehicle(client, generator, opened):
    assert client.post("/ui/open/vehicles/my_car").status_code == 200
    assert opened == [utils.get_vehicle_path(generator.vehicles_folder, "my_car")]


def test_ui_open_folders(client, generator, user_folder, opened):
    mods = os.path.join(str(user_folder), "current", "mods", "unpacked")
    assert client.post("/ui/open/vehicles").status_code == 200
    assert client.post("/ui/open/mods").status_code == 200
    assert opened == [generator.vehicles_folder, mods]
    assert os.path.isdir(generator.vehicles_folder) and os.path.isdir(mods)


def test_ui_open_existing_settings(client, settings_file, opened):
    content = settings_file.read_text(encoding="utf-8")
    assert client.post("/ui/open/settings").status_code == 200
    assert opened == [str(settings_file)]
    assert settings_file.read_text(encoding="utf-8") == content


def test_ui_open_creates_missing_settings(client, tmp_path, opened, monkeypatch):
    file = tmp_path / "new" / "settings.json"
    monkeypatch.setattr(api, "generator", TemplateCarGenerator(str(file)))
    assert client.post("/ui/open/settings").status_code == 200
    assert opened == [str(file)]
    assert Settings.model_validate(utils.read_json(str(file))) == Settings()


def test_ui_open_missing_path(tmp_path):
    with pytest.raises(HTTPException) as error:
        utils.ui_open(str(tmp_path / "missing"))
    assert error.value.status_code == 404
