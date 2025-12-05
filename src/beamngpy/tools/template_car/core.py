"""Core classes and functions."""

import os
import subprocess
from pathlib import Path
from beamngpy.beamng.filesystem import determine_binary
from .utils import (
    default_settings_file, get_beamng_install_path, get_vehicles_folder, get_vehicle_path, get_user_folder, read_json
)
from .optimization import generate_optimal_car
from .models import Settings, TemplateVehicle


class TemplateCarGenerator:
    """Template car generator."""

    def __init__(self, settings_file: str|None=None):
        """Initialize template car generator."""
        if settings_file:
            settings_file = os.path.abspath(settings_file)
        else:
            settings_file = default_settings_file()
        self.settings_file: str = settings_file

    @property
    def vehicles_folder(self) -> str:
        """Return path to vehicles folder."""
        return get_vehicles_folder(self.get_settings())

    @property
    def user_folder(self) -> str:
        """Return path to BeamNG user folder."""
        return get_user_folder(self.get_settings())
    
    @property
    def install_path(self) -> str:
        """Return path to BeamNG install folder."""
        return get_beamng_install_path(self.get_settings())

    def get_settings(self) -> Settings:
        """Read settings from file."""
        if os.path.isfile(self.settings_file):
            settings = read_json(self.settings_file)
        else:
            settings = {}
        return Settings.model_validate(settings)

    def get_vehicle_list(self) -> list[str]:
        """Return list of all vehicle IDs."""
        vehicle_list = []
        for file in os.listdir(self.vehicles_folder):
            if file.endswith(".json"):
                vehicle_list.append(file[:-5])  # Remove .json extension
        vehicle_list.sort()
        return vehicle_list

    def get_vehicle(self, vehicle_id: str) -> TemplateVehicle:
        """Return TemplateVehicle object by ID."""
        v = read_json(get_vehicle_path(self.vehicles_folder, vehicle_id))
        return TemplateVehicle.model_validate(v)

    def generate(self, vehicle_id: str, vehicle: TemplateVehicle) -> None:
        """Optimize and generate the vehicle mod and move it to BeamNG mods folder."""
        generate_optimal_car(vehicle_id, vehicle, self.user_folder, self.install_path)

    def play(self, vehicle_id: str, level_id: str) -> None:
        """Launch BeamNG with the generated vehicle mod."""
        install_path = Path(self.install_path)
        args = ["-level", level_id, "-vehicle", vehicle_id, "-userpath", self.user_folder]
        binary = determine_binary(install_path)
        subprocess.run([binary, *args], check=True)
