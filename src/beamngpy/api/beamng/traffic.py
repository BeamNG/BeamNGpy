from __future__ import annotations

from typing import TYPE_CHECKING, List

from beamngpy.types import StrDict

from .base import Api

if TYPE_CHECKING:
    from beamngpy.vehicle import Vehicle


class TrafficApi(Api):
    """
    An API for controlling the traffic

    Args:
        beamng: An instance of the simulator.
    """

    def start(self, participants: List[Vehicle]) -> None:
        """
        Enables traffic simulation for the given list of vehicles.

        Args:
            participants: List of vehicles that will be part of the simulation.
                        These vehicles need to be spawned beforehand and the
                        simulation will take control of them.
        """
        data: StrDict = dict(type="StartTraffic")
        data["participants"] = [p.vid for p in participants]
        self._send(data).ack("TrafficStarted")

    def spawn(
        self,
        max_amount: int | None = None,
        police_ratio: float = 0,
        extra_amount: int | None = None,
        parked_amount: int | None = None,
    ) -> None:
        """
        Enables traffic simulation with freshly spawned vehicles.

        Args:
            max_amount: The maximum allowed vehicles to spawn. If None, defaults to in-game settings.
            police_ratio: A number between 0.0 and 1.0 indicating the ratio of police vehicles in the traffic.
            extra_amount: The maximum amount of inactive vehicles to spawn and swap in and out of the traffic system.
                          If None, defaults to in-game settings.
            parked_amount: The maximum amount of parked vehicles to spawn. If None, defaults to in-game settings.
        """
        data: StrDict = dict(type="SpawnTraffic")
        data["max_amount"] = max_amount
        data["police_ratio"] = police_ratio
        data["extra_amount"] = extra_amount
        data["parked_amount"] = parked_amount
        self._send(data).ack("TrafficSpawned")

    def reset(self) -> None:
        """
        Resets (force teleports) all vehicles in the traffic away from the player.
        Useful for resolving traffic jam situations.

        """
        data = dict(type="ResetTraffic")
        self._send(data).ack("TrafficReset")

    def stop(self, stop: bool = False) -> None:
        """
        Stops the traffic simulation.

        Args:
            stop: Whether or not to stop the vehicles participating in
                traffic. If True, vehicles will come to a halt, if
                False, the AI will simply stop controlling the vehicle.
        """
        data: StrDict = dict(type="StopTraffic")
        data["stop"] = stop
        self._send(data).ack("TrafficStopped")
