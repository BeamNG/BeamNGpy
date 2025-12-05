"""Utility functions."""

import os
import json
import re
import platform
import subprocess
import platform
from fastapi import HTTPException

from .models import TemplateVehicle, Settings


def get_vehicle_path(vehicles_folder: str, vehicle_id: str) -> str:
    """Get the file path for a vehicle."""
    return os.path.join(vehicles_folder, f"{vehicle_id}.json")


def vehicle_exists(vehicles_folder: str, vehicle_id: str) -> bool:
    """Check if a vehicle file exists."""
    return os.path.exists(get_vehicle_path(vehicles_folder, vehicle_id))


def save_vehicle_file(vehicles_folder: str, vehicle_id: str, vehicle: TemplateVehicle) -> None:
    """Save vehicle to JSON file."""
    file = get_vehicle_path(vehicles_folder, vehicle_id)
    os.makedirs(os.path.dirname(file), exist_ok=True)
    with open(file, 'w') as f:
        json.dump(vehicle.model_dump(), f, indent=4)


def ui_open(path: str) -> None:
    """Open a file or folder using the system's default application."""
    path = os.path.abspath(path)
    if not os.path.exists(path):
        raise HTTPException(status_code=404, detail=f"Path not found: {path}")
    
    system = platform.system()
    if system == "Windows":
        if os.path.isdir(path):
            subprocess.run(["explorer", path], check=False)
        else:
            subprocess.run(["start", "", path], shell=True, check=True)
    elif system == "Darwin":
        subprocess.run(["open", path], check=True)
    else:  # Linux
        subprocess.run(["xdg-open", path], check=True)


def read_file(file):
    with open(file, 'r', encoding='utf-8') as f:
        return f.read()


def read_json(file):
    with open(file, 'r', encoding='utf-8') as f:
        return json.load(f)


def default_settings_file() -> str:
    return os.path.join(default_template_car_data_path(), "settings.json")


def get_user_folder(settings: Settings) -> str:
    user_folder = settings.user_folder
    if user_folder == "":
        return default_user_folder()
    return user_folder


def get_beamng_install_path(settings: Settings) -> str:
    beamng_install_path = settings.beamng_install_path
    if beamng_install_path == "":
        return default_beamng_install_path(settings)
    return beamng_install_path


def default_beamng_install_path(settings: Settings) -> str:
    bng_home = os.environ.get("BNG_HOME")
    if bng_home and os.path.exists(bng_home):
        return bng_home
    user_folder = get_user_folder(settings)
    p = os.path.abspath(os.path.join(user_folder, ".."))
    ini_file_choices = ["BeamNG.tech.ini", "BeamNG.drive.ini"]
    for ini_file in ini_file_choices:
        file = os.path.join(p, ini_file)
        if os.path.exists(file):
            with open(file, 'r') as f:
                for line in f:
                    if line.startswith("installPath = "):
                        return line.split(" = ")[1].strip()
    return "C:\\BeamNG" if platform.system() == "Windows" else "/BeamNG"


def get_vehicles_folder(settings: Settings) -> str:
    vehicles_folder = settings.vehicles_folder
    if vehicles_folder == "":
        return os.path.join(default_template_car_data_path(), "vehicles")
    return vehicles_folder


def default_beamng_data_path() -> str:
    if platform.system() == "Windows":
        location = os.environ['LOCALAPPDATA']
    else:
        location = os.path.expanduser("~/.local/share")
    return os.path.join(location, "BeamNG")


def default_user_folder():
    return os.path.join(default_beamng_data_path(), "BeamNG.tech")


def default_template_car_data_path() -> str:
    return os.path.join(default_beamng_data_path(), "TemplateCarGenerator")


def validate_id(id):
    pattern = r'^[a-z][a-z0-9_]*$'
    if not re.match(pattern, id):
        raise ValueError(f"Invalid ID: {id}. Must start with a lowercase letter and contain only lowercase letters, numbers, and underscores.")


def mod_exists(user_folder: str, vehicle_id: str) -> bool:
    mod_path = os.path.join(user_folder, "current", "mods", "unpacked", vehicle_id)
    return os.path.exists(mod_path)


def jbeam_template_dir() -> str:
    return os.path.join(assets_dir(), "template")


def web_dir() -> str:
    return os.path.join(assets_dir(), "web")


def assets_dir() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
