from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Optional, List, Union
from beamngpy.logging import BNGValueError
from beamngpy.types import Float3, StrDict

from .base import VehicleApi


class AIApi(VehicleApi):
    """
    An API class gathering AI-related functionality.

    Args:
        vehicle: An instance of a vehicle object.
    """

    def set_mode(self, mode: str) -> None:
        """
        Sets the desired mode of the simulator's built-in AI for this vehicle.
        Possible values are:

         * ``disabled``: Turn the AI off (default state)
         * ``random``: Drive from random points to random points on the map
         * ``traffic``: Act like a traffic vehicle
         * ``span``: Drive along the entire road network of the map
         * ``manual``: Drive to a specific waypoint, target set separately
         * ``chase``: Chase a target vehicle, target set separately
         * ``flee``: Flee from a vehicle, target set separately
         * ``stopping``: Make the vehicle come to a halt (AI disables itself once the vehicle stopped.)

        Note:
            Some AI methods automatically set appropriate modes, meaning a call
            to this method might be optional.

        Args:
            mode: The AI mode to set.
        """
        data = dict(type="SetAiMode")
        data["mode"] = mode
        self._send(data).ack("AiModeSet")

    def set_speed(self, speed: float, mode: str = "limit") -> None:
        """
        Sets the target speed for the AI in m/s. Speed can be maintained in two
        modes:

         * ``limit``: Drive speeds between 0 and the limit, as the AI
                        sees fit.
         * ``set``: Try to maintain the given speed at all times.

        Args:
            speed: The target speed in m/s.
            mode: The speed mode.
        """
        data: StrDict = dict(type="SetAiSpeed")
        data["speed"] = speed
        data["mode"] = mode
        self._send(data).ack("AiSpeedSet")

    def set_target(self, target: str, mode: str = "chase") -> None:
        """
        Sets the target to chase or flee. The target should be the ID of
        another vehicle in the simulation. The AI is automatically set to the
        given mode.

        Args:
            target: ID of the target vehicle as a string.
            mode: How the target should be treated. ``chase`` to chase the
                       target, ``flee`` to flee from it.
        """
        self.set_mode(mode)
        data = dict(type="SetAiTarget")
        data["target"] = target
        self._send(data).ack("AiTargetSet")

    def set_waypoint(self, waypoint: str) -> None:
        """
        Sets the waypoint the AI should drive to in manual mode. The AI gets
        automatically set to manual mode when this method is called.

        Args:
            waypoint: ID of the target waypoint as a string.
        """
        self.set_mode("manual")
        data = dict(type="SetAiWaypoint")
        data["target"] = waypoint
        self._send(data).ack("AiWaypointSet")

    def drive_using_waypoints(
        self,
        wp_target_list: List[str],
        wp_speeds: Optional[Dict[str, float]] = None,
        no_of_laps: int = 1,
        route_speed: Optional[float] = None,
        route_speed_mode: Optional[str] | None = None,
        drive_in_lane: bool = False,
        aggression: float = 0.3,
        avoid_cars: bool = False,
    ):
        """
        Sets a list of the waypoints the AI should drive to.

        Args:
            wp_target_list: A sequence of waypoint names to be used as succesive targets ex. wp_target_list = ['wp1', 'wp2']. Between any two consequitive waypoints a shortest path route will be followed.
            wp_speeds: Type: (key/value pairs, key: "node_name", value: speed, number in m/s)
                        Define target speeds for individual waypoints. The ai will try to meet this speed when at the given waypoint.
            no_of_laps: The number of laps if the path is a loop. If not specified, the ai will just follow the succesion of waypoints once.
            route_speed: A speed in m/s. To be used in tandem with ``route_speed_mode``.
            route_speed_mode: One of the following options.
                * ``limit``: the ai will not go above the ``route_speed``.
                * ``set``: the ai will try to always go at the speed defined by ``routeSpeed``.
            drive_in_lane: When True, the ai will keep on the correct side of the road on two way roads. This also affects path finding
                           in that when this option is active ai paths will traverse roads in the legal direction if possible.
            aggression: Value: 0.3-1. The aggression value with which the ai will drive the route.
                        At 1 the ai will drive at the limit of traction. A value of 0.3 would be considered normal every day driving, going shopping etc.
            avoid_cars: When True, the ai will be aware of (avoid crashing into) other vehicles on the map.
        """
        data = dict(type="DriveUsingPath")
        data["wpTargetList"] = wp_target_list
        data["wpSpeeds"] = wp_speeds
        data["noOfLaps"] = no_of_laps
        data["routeSpeed"] = route_speed
        data["routeSpeedMode"] = route_speed_mode
        data["driveInLane"] = drive_in_lane
        data["aggression"] = aggression
        data["avoidCars"] = avoid_cars
        self._send(data).ack("DriveUsingPath")

    def drive_in_lane(self, lane: bool) -> None:
        """
        Sets the drive in lane flag of the AI. If True, the AI only drives
        within the lane it can legally drive in.

        Args:
            lane: Lane flag to set.
        """
        data = dict(type="SetDriveInLane")
        data["lane"] = "on" if lane else "off"
        self._send(data).ack("AiDriveInLaneSet")

    def set_line(
        self, line: List[Dict[str, Float3 | float]], cling: bool = True
    ) -> None:
        """
        Makes the AI follow a given polyline. The line is specified as a list
        of dictionaries where each dictionary has a ``pos`` entry specifying the
        supposed position as an ``(x, y, z)`` triplet and a ``speed`` entry
        specifying the speed in m/s.

        Args:
            line: Polyline as list of dicts as described above.
            cling: Whether or not to align the ``z`` coordinate of the polyline to the ground.
        """
        data: StrDict = dict(type="SetAiLine")
        data["line"] = line
        data["cling"] = cling
        return self._send(data).ack("AiLineSet")

    def set_script(self, script: List[Dict[str, float]], cling: bool = True) -> None:
        """
        Makes the vehicle follow a given "script" -- a script being a list of
        timestamped positions defining where a vehicle should be at what time.
        This can be used to make the vehicle drive a long a polyline with speed
        implicitly expressed in the time between points.

        Args:
            script: A list of nodes in the script. Each node is expected to be a
                    dict-like that has ``x``, ``y``, and ``z`` entries for the supposed
                    position of the vehicle, and a ``t`` entry for the time of the
                    node along the path. Time values are in seconds relative to the
                    time when script playback is started.
            cling: A flag that makes the simulator cling z-coordinates to the ground.
                   Since computing z-coordinates in advance without knowing the level
                   geometry can be cumbersome, this flag is used to automatically set
                   z-coordinates in the script to the ground height. Defaults to True.

        Notes:
            The AI follows the given script the best it can. It cannot drive
            along scripts that would be physically impossible, e.g. specifying
            a script with points A & B one kilometer apart and giving it a
            a second between those points will make the AI drive from A to B as
            fast as it can, but unlikely to reach it in the given time.
            Furthermore, if the AI falls behind schedule, it will start
            skipping points in the script in an effort to make up for
            lost time.

        Raises:
            BNGValueError: If the script has fewer than three nodes, the
                           minimum length of a script.
        """
        if len(script) < 3:
            raise BNGValueError("AI script must have at least 3 nodes.")

        data: StrDict = dict(type="SetAiScript")
        data["script"] = script
        data["cling"] = cling
        self._send(data).ack("AiScriptSet")

    def import_script_ai_file(
        self, file_path: Union[str, Path]
    ) -> List[Dict[str, float]]:
        """
        Import a script AI file from BeamNG and return it in BeamNGpy script format.

        Automatically looks in the BeamNG user folder if only a filename is provided.

        Args:
            file_path: Path to the JSON file to import. If just a filename is provided,
                    it will automatically look in the BeamNG user folder first.

            Returns:
                Script data in format: [{"x": ..., "y": ..., "z": ..., "t": ...}, ...]

            Example:
                    # //script location is userfolder
                        script = vehicle.ai.import_script_ai_file("<script_AI_editor_name>.json")
                        vehicle.ai.set_script(script)

        """
        file_path = Path(file_path)

        # If only a filename is provided, try to find it in the user folder first
        if not file_path.is_absolute() and len(file_path.parts) == 1:
            # Try to get the user folder from the connected BeamNG instance
            if (
                hasattr(self._vehicle, "bng")
                and self._vehicle.bng
                and hasattr(self._vehicle.bng, "user_with_version")
            ):
                user_folder_path = Path(self._vehicle.bng.user_with_version) / file_path
                if user_folder_path.exists():
                    file_path = user_folder_path

        if not file_path.exists():
            raise FileNotFoundError(f"Script file not found: {file_path}")

        with open(file_path, "r") as f:
            data = json.load(f)

        # All BeamNG.tech path files have the same structure with "path" array
        if "path" not in data:
            raise ValueError(
                f"File {file_path} is not a valid BeamNG.tech path file (missing 'path' array)"
            )

        path_data = data["path"]

        if not path_data:
            raise ValueError(f"No path data found in {file_path}")

        # Check if we have time values or need to calculate them
        has_time = any(
            "t" in waypoint for waypoint in path_data[:3]
        )  # Check first 3 points

        script = []

        if has_time:
            # drawn_path_time.json and recorded_path.json - use existing time values
            for waypoint in path_data:
                script_point = {
                    "x": float(waypoint["x"]),
                    "y": float(waypoint["y"]),
                    "z": float(waypoint["z"]),
                    "t": float(waypoint["t"]),
                }
                script.append(script_point)
        else:
            # drawn_path_velocity.json - ignore velocity, calculate time from distance
            current_time = 0.0

            for i, waypoint in enumerate(path_data):
                script_point = {
                    "x": float(waypoint["x"]),
                    "y": float(waypoint["y"]),
                    "z": float(waypoint["z"]),
                    "t": current_time,
                }
                script.append(script_point)

                # Calculate time to next waypoint based on distance
                if i < len(path_data) - 1:
                    next_waypoint = path_data[i + 1]

                    # Calculate distance to next point
                    dx = float(next_waypoint["x"]) - float(waypoint["x"])
                    dy = float(next_waypoint["y"]) - float(waypoint["y"])
                    dz = float(next_waypoint["z"]) - float(waypoint["z"])
                    distance = (dx**2 + dy**2 + dz**2) ** 0.5

                    # Use distance-based timing (assume ~15 m/s average speed)
                    time_increment = distance / 15.0 if distance > 0 else 1.0
                    current_time += time_increment

        if len(script) < 3:
            raise ValueError(
                f"Script must have at least 3 waypoints, got {len(script)}"
            )

        return script

    def set_aggression(self, aggr: float) -> None:
        data: StrDict = dict(type="SetAiAggression")
        data["aggression"] = aggr
        self._send(data).ack("AiAggressionSet")

    def start_recording(self) -> None:
        data = dict(type="StartRecording")
        self._send(data).ack("CompletedStartRecording")

    def stop_recording(self, filename) -> None:
        data = dict(type="StopRecording")
        data["filename"] = filename
        self._send(data).ack("CompletedStopRecording")

    def execute_script(
        self,
        script,
        cling: bool = True,
        start_delay: float = 0.0,
        no_reset: bool = False,
    ) -> None:
        data: StrDict = dict(type="ExecuteScript")
        data["script"] = script
        data["cling"] = cling
        data["startDelay"] = start_delay
        data["noReset"] = no_reset
        self._send(data).ack("CompletedExecuteScript")

    def get_initial_spawn_position_orientation(self, script):
        data = dict(type="GetInitialSpawnPositionOrientation")
        data["script"] = script
        return self._send(data).recv()["data"]
