from __future__ import annotations

from typing import TYPE_CHECKING, List

from beamngpy.types import StrDict

from .base import Api

if TYPE_CHECKING:
    from beamngpy.vehicle import Vehicle


class TrafficApi(Api):
    def start(self, participants: List[Vehicle]) -> None:
        """
        Enables traffic simulation for the given list of vehicles.

        Args:
            participants: List of vehicles that will be part of the simulation.
                        These vehicles need to be spawned beforehand and the
                        simulation will take control of them.
        """
        data: StrDict = dict(type='StartTraffic')
        data['participants'] = [p.vid for p in participants]
        self._send(data).ack('TrafficStarted')

    def stop(self, stop: bool = False) -> None:
        """
        Stops the traffic simulation.

        Args:
            stop: Whether or not to stop the vehicles participating in
                traffic. If True, vehicles will come to a halt, if
                False, the AI will simply stop controlling the vehicle.
        """
        data: StrDict = dict(type='StopTraffic')
        data['stop'] = stop
        self._send(data).ack('TrafficStopped')
