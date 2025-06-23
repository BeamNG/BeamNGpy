from __future__ import annotations

from .base import Api


class PlatoonApi(Api):
    """
    An API for vehicle platooning formation.

    Args:
        vehicle: An instance of a vehicle object.
    """

    def load(
        self,
        leaderID: str,
        veh1ID: str,
        veh2ID: str,
        veh3ID: str,
        speed: float,
        inputFlag: bool,
    ) -> None:
        """
        A function for forming the platoon that starts the platoon with one leader and three followers

        Args:
            leaderID: An instance of a vehicle object of the platoon's leader
            follower1ID=An instance of a vehicle object of the following vehicle
            follower2ID=An instance of a vehicle object of the following vehicle
            follower3ID=An instance of a vehicle object of the following vehicle
            speed=target speed,
            debugFlag=inputFlag
        """

        data = dict(
            type="LoadPlatoon",
            leaderID=leaderID,
            follower1ID=veh1ID,
            follower2ID=veh2ID,
            follower3ID=veh3ID,
            speed=speed,
            debugFlag=inputFlag,
        )

        self._send(data).ack("platoonLoaded")

    def launch_platoon(
        self, leaderID: str, mode: int, speed: float, inputFlag: bool
    ) -> None:
        """
        A function for launching the platoon by setting the leader's driving mode(0: manual, 1: Span, 2: Traffic) and setting its target speed

        Args:
            leaderID: An instance of a vehicle object of the platoon's leader
            mode: driving mode selected
            speed: target speed
            debugFlag=inputFlag
        """

        data = dict(
            type="LaunchPlatoon",
            leaderID=leaderID,
            mode=mode,
            speed=speed,
            debugFlag=inputFlag,
        )

        self._send(data).ack("platoonLaunched")

    def join(self, leaderID: str, vehID: str, speed: float, inputFlag: bool) -> None:
        """
         A function for vehicles to join the platoon at the end of the platoon

        Args:
            leaderID: An instance of a vehicle object of the platoon's leader
            vehID: An instance of a vehicle object of the external vehicle joining ID
            speed: speed
            debugFlag=inputFlag
        """

        data = dict(
            type="JoinPlatoon",
            leaderID=leaderID,
            vid=vehID,
            speed=speed,
            debugFlag=inputFlag,
        )

        self._send(data).ack("platoonJoined")

    def leave(self, leaderID: str, vehID: str, inputFlag: bool) -> None:
        """
        A function for vehicles to leave the platoon

        Args:
            leaderID: An instance of a vehicle object of the platoon's leader
            vehID: An instance of a vehicle object of the  vehicle ID leaving the platoon of the vehicle leaving
            debugFlag=inputFlag
        """

        data = dict(
            type="LeavePlatoon", leaderID=leaderID, vid=vehID, debugFlag=inputFlag
        )

        self._send(data).ack("platoonLeft")

    def join_middle(
        self, leaderID: str, vehPID: str, vehID: str, speed: float, inputFlag: bool
    ) -> None:
        """
        A function for vehicles to join in the middle of the platoon

        Args:
            leaderID: An instance of a vehicle object of the platoon's leader.
            vehPID: An instance of a vehicle object of the vehicle in the platoon the external is joining infront of
            vehID: An instance of a vehicle object of the vehicle joining
            speed: speed
            debugFlag: debugFlag
        """

        data = dict(
            type="JoinMiddlePlatoon",
            leaderID=leaderID,
            vPid=vehPID,
            vid=vehID,
            speed=speed,
            debugFlag=inputFlag,
        )

        self._send(data).ack("platoonJoined")
