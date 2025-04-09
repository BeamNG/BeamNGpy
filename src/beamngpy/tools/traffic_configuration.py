from __future__ import annotations

import json
from logging import DEBUG, getLogger
from pathlib import Path

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.logging import LOGGER_ID


class TrafficConfig:
    """
    Creates and starts a scenario based on a traffic configuration save file.
    Loads level with a set of vehicles/props with unique AI settings.

    Vehicle objects, created by this tool, should be accessed like so: ``TrafficConfig.vehicles[<vehicle_name>]``

    Args:
            bng: The BeamNGpy instance, with which to communicate to the simulation.
            config_path: The BeamNG local path to the wanted traffic configuration file (example: "/traffic.json")

    """

    def __init__(self, bng: BeamNGpy, config_path: str):
        self.logger = getLogger(f"{LOGGER_ID}.TrafficConfig")
        self.logger.setLevel(DEBUG)

        # load json as dict
        with open(self.__merge_dir(bng.user_with_version, config_path), "r", encoding="utf-8") as f:
            self.json = json.load(f)

        scenario = Scenario(self.json["level"], self.json["name"])
        self.vehicles = {}

        # add vehicles/props
        for veh_data in self.json["vehicles"]:
            self.vehicles[veh_data["name"]] = Vehicle(
                veh_data["name"], veh_data["model"], part_config=veh_data["config"]
            )
            scenario.add_vehicle(
                self.vehicles[veh_data["name"]],
                veh_data["home"]["pos"],
                self.__change_rotation(veh_data["home"]["rot"]),
            )

        self.logger.info("Loaded traffic configuration.")

        scenario.make(bng)
        bng.scenario.load(scenario)
        bng.scenario.start()

        # set AI settings and enable AI
        for veh_data in self.json["vehicles"]:
            if (
                veh_data["vehType"] in ["Car", "Truck", "Automation"]
                and veh_data["name"] in self.vehicles.keys()
            ):
                if veh_data["aiType"] == "basic":
                    self.vehicles[veh_data["name"]].ai_set_aggression(
                        veh_data["aiData"]["aggression"]
                    )
                    self.vehicles[veh_data["name"]].ai_drive_in_lane(
                        veh_data["aiData"]["driveInLane"]
                    )
                    self.vehicles[veh_data["name"]].ai_set_speed(
                        veh_data["aiData"]["speed"], "limit"
                    )
                elif veh_data["aiType"] == "script":
                    self.vehicles[veh_data["name"]].ai.execute_script(
                        self.__convert_json_trajectory(
                            self.__merge_dir(bng.user_with_version, veh_data["aiData"]["scriptFile"])
                        )
                    )
                self.vehicles[veh_data["name"]].ai_set_mode(veh_data["aiMode"])

        self.logger.info("Started scenario with AI")

    def __merge_dir(self, user: str, other: str):
        return Path(user) / other.lstrip("/\\")

    def __change_rotation(self, rot):
        # rotate 180 regrees to face forward
        w1, x1, y1, z1 = rot
        w2, x2, y2, z2 = (0, 1, 0, 0)
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return (w, x, y, z)

    def __convert_json_trajectory(self, file_path: str):
        with open(file_path, "r") as file:
            data = json.load(file)

        # Extract the path
        script = [
            {"t": node["t"], "x": node["x"], "y": node["y"], "z": node["z"]}
            for node in data["path"]
        ]

        return script
