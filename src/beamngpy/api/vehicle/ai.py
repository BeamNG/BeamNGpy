from __future__ import annotations

from typing import Dict, List

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
