from __future__ import annotations

from beamngpy.types import StrDict

from .base import Api


class SystemApi(Api):
    """
    An API for getting info about the host system running the simulator.

    Args:
        beamng: An instance of the simulator.
    """

    def get_info(
        self, os: bool = True, cpu: bool = False, gpu: bool = False, power: bool = False
    ) -> StrDict:
        """
        Returns the information about the host's system.

        Args:
            os: Whether to include information about the operating system of the host.
            cpu: Whether to include information about the CPU of the host.
            gpu: Whether to include information about the GPU of the host.
            power: Whether to include information about the power options of the host.
        """
        data = dict(type="GetSystemInfo", os=os, cpu=cpu, gpu=gpu, power=power)
        return self._send(data).recv()
