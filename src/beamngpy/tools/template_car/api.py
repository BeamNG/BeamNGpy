"""REST API for the template car application."""

import os
import json
import logging
import send2trash
from fastapi import FastAPI, HTTPException, status
from fastapi.responses import FileResponse, StreamingResponse

from .models import TemplateVehicle, Settings, VehicleListItem, Message
from .utils import (
    vehicle_exists,
    save_vehicle_file,
    ui_open,
    get_vehicle_path,
    mod_exists,
    web_dir,
    ensure_directory,
    write_settings,
)
from .core import TemplateCarGenerator
from .optimization import generate_optimal_car
from .streaming import stream_from_print

logger = logging.getLogger(__name__)

app = FastAPI(title="Template Car API", version="1.0.0")
web = web_dir()
generator = TemplateCarGenerator(os.environ.get('TEMPLATE_CAR_SETTINGS_FILE'))


@app.get("/", tags=["Web"])
def web_application():
    """Serve web app."""
    return FileResponse(f"{web}/index.html")


@app.get("/static/{path:path}", tags=["Web"])
def serve_static_file(path: str):
    """Serve static files from web/static folder."""
    assert '..' not in path, "Path must not contain .."
    return FileResponse(f"{web}/static/{path}")


@app.get("/vehicles", tags=["Vehicles"])
def list_vehicles() -> list[VehicleListItem]:
    """List all available vehicles with their IDs, names and flag if the mod is generated."""
    vehicles = []
    vehicles_folder = generator.vehicles_folder
    user_folder = generator.user_folder
    if os.path.exists(vehicles_folder):
        for vehicle_id in generator.get_vehicle_list():
            try:
                vehicle = generator.get_vehicle(vehicle_id)
                vehicles.append(VehicleListItem.model_validate(
                    {"id": vehicle_id, "name": vehicle.name, "is_generated": mod_exists(user_folder, vehicle_id)})
                )
            except Exception as e:
                logger.error(f"Error loading vehicle {vehicle_id}: {e}")
                continue  # Skip invalid files
    return vehicles


@app.post("/vehicles/{vehicle_id}", status_code=status.HTTP_201_CREATED, tags=["Vehicles"])
def create_vehicle(vehicle_id: str, vehicle: TemplateVehicle) -> TemplateVehicle:
    """Create a new vehicle with given parameters."""
    if vehicle_exists(generator.vehicles_folder, vehicle_id):
        raise HTTPException(status_code=409, detail=f"Vehicle '{vehicle_id}' already exists")

    save_vehicle_file(generator.vehicles_folder, vehicle_id, vehicle)
    return vehicle


@app.get("/vehicles/{vehicle_id}", tags=["Vehicles"])
def get_vehicle(vehicle_id: str) -> TemplateVehicle:
    """Get vehicle parameters."""
    try:
        vehicle = generator.get_vehicle(vehicle_id)
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail=f"Vehicle '{vehicle_id}' not found")
    return vehicle


@app.put("/vehicles/{vehicle_id}", tags=["Vehicles"])
def update_vehicle(vehicle_id: str, vehicle: TemplateVehicle) -> TemplateVehicle:
    """Update existing vehicle parameters."""
    if not vehicle_exists(generator.vehicles_folder, vehicle_id):
        raise HTTPException(status_code=404, detail=f"Vehicle '{vehicle_id}' not found")

    save_vehicle_file(generator.vehicles_folder, vehicle_id, vehicle)
    return vehicle


@app.delete("/vehicles/{vehicle_id}", status_code=status.HTTP_204_NO_CONTENT, tags=["Vehicles"])
def delete_vehicle(vehicle_id: str) -> None:
    """Delete a vehicle file."""
    if not vehicle_exists(generator.vehicles_folder, vehicle_id):
        raise HTTPException(status_code=404, detail=f"Vehicle '{vehicle_id}' not found")

    send2trash.send2trash(get_vehicle_path(generator.vehicles_folder, vehicle_id))


@app.post("/vehicles/{vehicle_id}/generate", tags=["Vehicles"])
async def generate_vehicle_mod(vehicle_id: str) -> Message:
    """Optimize and generate the vehicle mod and move it to BeamNG mods folder."""
    try:
        vehicle = generator.get_vehicle(vehicle_id)
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail=f"Vehicle '{vehicle_id}' not found")
    def error_string(e):
        return f"\n--- ERROR ---\nGeneration failed: {str(e)}"

    def run_job(file):
        generate_optimal_car(vehicle_id, vehicle, generator.user_folder, generator.install_path, file=file)

    return StreamingResponse(stream_from_print(run_job, error_string), media_type="text/plain")


@app.post("/vehicles/{vehicle_id}/play/{level_id}", tags=["Vehicles"])
def play_vehicle_mod(vehicle_id: str, level_id: str) -> Message:
    """Launch BeamNG with the generated vehicle mod."""
    try:
        generator.play(vehicle_id, level_id)
        return Message(message=f"BeamNG launched with vehicle '{vehicle_id}' on level '{level_id}'")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"BeamNG launch failed: {str(e)}")


@app.get("/settings", tags=["Settings"])
def get_settings() -> Settings:
    """Get current application settings."""
    return generator.get_settings()


@app.put("/settings", tags=["Settings"])
def update_settings(settings: Settings) -> Settings:
    """Update application settings."""
    write_settings(generator.settings_file, settings)
    return settings


@app.get("/schemas", tags=["Settings"])
def get_schemas() -> dict[str, dict]:
    """Get JSON schemas for all data models."""
    return {
        "TemplateVehicle": TemplateVehicle.model_json_schema(),
        "Settings": Settings.model_json_schema()
    }


@app.post("/ui/open/vehicles/{vehicle_id}", tags=["Actions"])
def open_vehicle_file(vehicle_id: str) -> Message:
    """Open vehicle file in default editor."""
    vehicle_path = get_vehicle_path(generator.vehicles_folder, vehicle_id)
    ui_open(vehicle_path)
    return Message(message=f"Opened vehicle: {vehicle_id}")


@app.post("/ui/open/vehicles", tags=["Actions"])
def open_vehicles_folder() -> Message:
    """Open vehicles folder in file explorer."""
    folder = generator.vehicles_folder
    ensure_directory(folder)
    ui_open(folder)
    return Message(message="Opened vehicles folder")


@app.post("/ui/open/settings", tags=["Actions"])
def open_settings_file() -> Message:
    """Open settings file in default editor."""
    file = generator.settings_file
    if not os.path.exists(file):
        write_settings(file, generator.get_settings())
    ui_open(generator.settings_file)
    return Message(message="Opened settings file")


@app.post("/ui/open/mods", tags=["Actions"])
def open_mods_folder() -> Message:
    """Open BeamNG mods folder in file explorer."""
    mods_path = os.path.join(generator.user_folder, "current", "mods", "unpacked")
    ensure_directory(mods_path)
    ui_open(mods_path)
    return Message(message="Opened mods folder")
