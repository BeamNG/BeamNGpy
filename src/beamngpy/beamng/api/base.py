from __future__ import annotations

from typing import TYPE_CHECKING

from beamngpy.logging import generate_docstring

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy

class Api:
    def __init__(self, beamng: BeamNGpy):
        self._beamng = beamng
        self._send = beamng.send
        self._message = beamng.message
        self._logger = beamng.logger
        self.__doc__ = generate_docstring(self)
