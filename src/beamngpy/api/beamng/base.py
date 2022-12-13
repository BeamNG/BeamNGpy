from __future__ import annotations

from typing import TYPE_CHECKING

from beamngpy.logging import _generate_docstring

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy


class Api:
    """
    A base API class from which all the API communicating with the simulator derive.

    Args:
        beamng: An instance of the simulator.
    """

    def __init__(self, beamng: BeamNGpy):
        self._beamng = beamng
        self._send = beamng._send
        self._message = beamng._message
        self._logger = beamng.logger
        self.__doc__ = _generate_docstring(self)
