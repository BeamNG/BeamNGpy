from __future__ import annotations

from .base import Api


def toLuaTable (py_dict):
    lua_items = []
    for k, v in py_dict.items():
        if isinstance(v, str):
            v = f'"{v}"'
        elif isinstance(v, bool):
            v = 'true' if v else 'false'
        lua_items.append(f'{k} = {v}')
    return '{' + ', '.join(lua_items) + '}'

class PlatoonApi(Api):
    """
    A base API class from which all the API communicating with a vehicle derive.

    Args:
        vehicle: An instance of a vehicle object.
    """

    def form(self, leaderid: str, vehid: str, sp: float, inputFlag: bool) -> None:
        """
        forms platoon

        Args:
            vehicle: An instance of a vehicle object.
        """
        data = dict(
            type="FormPlatoon"
        )
        data["leaderID"] = leaderid  # The leader’s ID
        data["vid"] = vehid  # The vehicle’s ID
        data["speed"] = sp
        data["debugFlag"] = inputFlag
        self._send(data).ack("Platoonformed")
        log_msg = "Formed platoon."
        self._logger.info(log_msg)

    def load (self, leaderID: str, veh1ID: str, veh2ID:str, veh3ID:str, speed: float, inputFlag: bool) ->None:
        print("loaded platoon")
        
        

        data = dict(
            type="LoadPlatoon",
            leaderID=leaderID,
            follower1ID=veh1ID,
            follower2ID = veh2ID,
            follower3ID = veh3ID,
            speed=speed,
            debugFlag=inputFlag
        )

        print(data)
        self._send(data).ack("platoonLoaded")

    def setSpeed (self, leaderID: str, speed: float, inputFlag: bool) ->None:
        print("changed speed")
        

        data = dict(
            type="PlatoonSpeed",
            leaderID=leaderID,
            speed=speed,
            debugFlag=inputFlag
        )

        print(data)
        self._send(data).ack("changedSpeed")

    def join(self, leaderID: str, vehID: str, speed:float, inputFlag: bool) -> None:
        """
        join platoon

        Args:
            vehicle: An instance of a vehicle object.
        """
  
        data = dict(
            type="JoinPlatoon",
            leaderID=leaderID,
            vid=vehID,
            speed=speed,
            debugFlag=inputFlag
        )

        self._send(data).ack("platoonJoined")
     

    def leave(self, leaderID: str, vehID: str, inputFlag: bool) -> None:
        """
        leave platoon

        Args:
            vehicle: An instance of a vehicle object.
        """

        data = dict(
            type="LeavePlatoon",
            leaderID=leaderID,
            vid=vehID,
            debugFlag=inputFlag
        )

        self._send(data).ack("platoonLeft")
   

    def joinMiddle(self, leaderID: str, vehPID:str, vehID: str, speed:float, inputFlag: bool) -> None:
        """
        join platoon

        Args:
            vehicle: An instance of a vehicle object.
        """

        data = dict(
            type="JoinMiddlePlatoon",
            leaderID=leaderID,
            vPid = vehPID,
            vid=vehID,
            speed=speed,
            debugFlag=inputFlag
        )

        self._send(data).ack("platoonJoined")
       
    def end(self) -> None:
        """
        Ends platoon.
        """
        data = dict(type="EndPlatoon")
        self._send(data).ack("Platoonended")
        self._logger.info("Ended Platoon.")

    def reassign(self, leaderid: str) -> None:
        """
        reassign leader platoon

        Args:
            leader vehicle: An instance of a vehicle object.
        """
        data = dict(
            type="ReassignLeaderNFSA"
        )
        data["leaderID"] = leaderid  # The leader’s ID
        self._send(data).ack("LeaderReassigned")
        log_msg = "Leader reassigned."
        self._logger.info(log_msg)

    def leaderexit(self, leaderid: str) -> None:
        """
        leader exits platoon

        Args:
            leader vehicle: An instance of a vehicle object.
        """
        data = dict(
            type="leaderExitPlatoon"
        )
        data["leaderID"] = leaderid  # The leader’s ID
        self._send(data).ack("leaderExitPlatoon")
        log_msg = "Leader exit."
        self._logger.info(log_msg)

    def updateLeader(self, leaderid: str) -> None:
        """
        leader is updated in platoon

        Args:
            leader vehicle: An instance of a vehicle object.
        """
        data = dict(
            type="UpdateLeaderToFollow"
        )
        data["leaderID"] = leaderid  # The leader’s ID
        self._send(data).ack("updatedLeader")
        log_msg = "Leader updated."
        self._logger.info(log_msg)