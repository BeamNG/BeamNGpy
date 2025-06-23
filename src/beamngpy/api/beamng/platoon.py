from __future__ import annotations

from .base import Api
from beamngpy.vehicle import Vehicle


class PlatoonApi(Api):
    """
    An API for vehicle platooning formation.

    Args:
        beamng: An instance of the simulator.
    """

    def load(
        self,
        leader: Vehicle | str,
        follower1: Vehicle | str,
        follower2: Vehicle | str | None,
        follower3: Vehicle | str | None,
        speed: float,
        debug: bool = False,
    ) -> None:
        """
        A function for forming the platoon that starts the platoon with one leader and three followers.

        Args:
            leader: An instance of a vehicle object of the platoon's leader.
            follower1: An instance of a vehicle object of the following vehicle.
            follower2: An instance of a vehicle object of the following vehicle.
            follower3: An instance of a vehicle object of the following vehicle.
            speed: Target speed in m/s.
            debug: Debugging flag.
        """
        leaderID = leader.vid if isinstance(leader, Vehicle) else leader
        follower1ID = follower1.vid if isinstance(follower1, Vehicle) else follower1
        follower2ID = follower2.vid if isinstance(follower2, Vehicle) else follower2
        follower3ID = follower3.vid if isinstance(follower3, Vehicle) else follower3
        data = dict(
            type="LoadPlatoon",
            leaderID=leaderID,
            follower1ID=follower1ID,
            follower2ID=follower2ID,
            follower3ID=follower3ID,
            speed=speed,
            debugFlag=debug,
        )
        self._send(data).ack("platoonLoaded")

    def launch_platoon(
        self, leader: Vehicle | str, mode: int, speed: float, debug: bool = False
    ) -> None:
        """
        A function for launching the platoon by setting the leader's driving mode(0: manual, 1: Span, 2: Traffic) and setting its target speed.

        Args:
            leader: An instance of a vehicle object of the platoon's leader.
            mode: Driving mode selected.
            speed: Target speed in m/s.
            debug: Debugging flag.
        """
        leaderID = leader.vid if isinstance(leader, Vehicle) else leader
        data = dict(
            type="LaunchPlatoon",
            leaderID=leaderID,
            mode=mode,
            speed=speed,
            debugFlag=debug,
        )
        self._send(data).ack("platoonLaunched")

    def join(
        self,
        leader: Vehicle | str,
        veh: Vehicle | str,
        speed: float,
        debug: bool = False,
    ) -> None:
        """
        A function for vehicles to join the platoon at the end of the platoon.

        Args:
            leader: An instance of a vehicle object of the platoon's leader.
            veh: An instance of a vehicle object of the external vehicle joining.
            speed: Target speed in m/s.
            debug: Debugging flag.
        """
        leaderID = leader.vid if isinstance(leader, Vehicle) else leader
        vid = veh.vid if isinstance(veh, Vehicle) else veh
        data = dict(
            type="JoinPlatoon",
            leaderID=leaderID,
            vid=vid,
            speed=speed,
            debugFlag=debug,
        )
        self._send(data).ack("platoonJoined")

    def leave(
        self, leader: Vehicle | str, veh: Vehicle | str, debug: bool = False
    ) -> None:
        """
        A function for vehicles to leave the platoon.

        Args:
            leader: An instance of a vehicle object of the platoon's leader.
            veh: An instance of a vehicle object of the vehicle leaving the platoon.
            debug: Debugging flag.
        """
        leaderID = leader.vid if isinstance(leader, Vehicle) else leader
        vid = veh.vid if isinstance(veh, Vehicle) else veh
        data = dict(type="LeavePlatoon", leaderID=leaderID, vid=vid, debugFlag=debug)
        self._send(data).ack("platoonLeft")

    def join_middle(
        self, leader: str, veh_platoon: str, veh: str, speed: float, debug: bool = False
    ) -> None:
        """
        A function for vehicles to join in the middle of the platoon.

        Args:
            leader: An instance of a vehicle object of the platoon's leader.
            veh_platoon: An instance of a vehicle object of the vehicle in the platoon the external is joining infront of.
            veh: An instance of a vehicle object of the vehicle joining.
            speed: Target speed in m/s.
            debug: Debugging flag.
        """
        leaderID = leader.vid if isinstance(leader, Vehicle) else leader
        vPid = veh_platoon.vid if isinstance(veh_platoon, Vehicle) else veh_platoon
        vid = veh.vid if isinstance(veh, Vehicle) else veh
        data = dict(
            type="JoinMiddlePlatoon",
            leaderID=leaderID,
            vPid=vPid,
            vid=vid,
            speed=speed,
            debugFlag=debug,
        )
        self._send(data).ack("platoonJoined")
