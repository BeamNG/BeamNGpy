from __future__ import annotations

from .base import Api
from beamngpy.vehicle import Vehicle


class PlatoonApi(Api):
    """
    An API for vehicle platooning formation.

    Args:
        beamng: An instance of the simulator.
    """

    def create(self, leader: Vehicle | str, follower: Vehicle | str) -> int:
        """
        Create a platoon with a leader and one follower.

        Args:
            leader: Platoon leader vehicle.
            follower: First follower vehicle.

        Returns:
            The platoon id assigned by the simulator.
        """
        leaderID = leader.vid if isinstance(leader, Vehicle) else leader
        followerID = follower.vid if isinstance(follower, Vehicle) else follower
        data = dict(
            type="CreatePlatoon",
            leaderID=leaderID,
            followerID=followerID,
        )
        resp = self._send(data).recv("CreatePlatoon")
        return resp["platoonId"]

    def join(
        self,
        platoon_id: int,
        follower: Vehicle | str,
        index: int | None = None,
    ) -> None:
        """
        Add a vehicle to a platoon.

        Args:
            platoon_id: Platoon id returned by :meth:`create`.
            follower: Vehicle joining the platoon.
            index: 0-based insertion index (0=new leader, 1=first follower,
                omit to append at the end).
        """
        followerID = follower.vid if isinstance(follower, Vehicle) else follower
        data = dict(type="JoinPlatoon", platoonId=platoon_id, followerID=followerID)
        if index is not None:
            data["index"] = index
        self._send(data).ack("JoinPlatoon")

    def split(self, platoon_id: int, vehicle: Vehicle | str) -> int:
        """
        Split a platoon at a vehicle, forming a second platoon behind it.

        Args:
            platoon_id: Platoon id returned by :meth:`create`.
            vehicle: Vehicle at which to split the platoon.

        Returns:
            The platoon id of the new rear platoon.
        """
        vehicleID = vehicle.vid if isinstance(vehicle, Vehicle) else vehicle
        data = dict(
            type="SplitPlatoon",
            platoonId=platoon_id,
            vehicleID=vehicleID,
        )
        resp = self._send(data).recv("SplitPlatoon")
        return resp["platoonId"]

    def launch(self, platoon_id: int, leader_mode: int, speed: float) -> None:
        """
        Launch a platoon by setting the leader's driving mode and target speed.

        Args:
            platoon_id: Platoon id returned by :meth:`create`.
            leader_mode: Driving mode (0: manual, 1: span, 2: traffic).
            speed: Target speed in m/s.
        """
        data = dict(
            type="Launch",
            platoonId=platoon_id,
            leaderMode=leader_mode,
            speed=speed,
        )
        self._send(data).ack("Launch")

    def change_speed(self, platoon_id: int, speed: float) -> None:
        """
        Change the commanded speed of a launched platoon.

        Args:
            platoon_id: Platoon id returned by :meth:`create`.
            speed: Target speed in m/s.
        """
        data = dict(type="ChangePlatoonSpeed", platoonId=platoon_id, speed=speed)
        self._send(data).ack("ChangePlatoonSpeed")

    def leave(self, platoon_id: int, vehicle: Vehicle | str) -> None:
        """
        Remove a vehicle from a platoon.

        Args:
            platoon_id: Platoon id returned by :meth:`create`.
            vehicle: Vehicle leaving the platoon.
        """
        vehicleID = vehicle.vid if isinstance(vehicle, Vehicle) else vehicle
        data = dict(
            type="LeavePlatoon",
            platoonId=platoon_id,
            vehicleID=vehicleID,
        )
        self._send(data).ack("LeavePlatoon")

    def disband(self, platoon_id: int) -> None:
        """
        Disband a platoon and unload platooning from all vehicles.

        Args:
            platoon_id: Platoon id returned by :meth:`create`.
        """
        data = dict(type="DisbandPlatoon", platoonId=platoon_id)
        self._send(data).ack("DisbandPlatoon")
