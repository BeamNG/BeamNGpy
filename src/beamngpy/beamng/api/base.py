from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy


class Api:
    def __init__(self, beamng: BeamNGpy):
        self.beamng = beamng
        self.send = beamng.send
        self.message = beamng.message
        self.logger = beamng.logger
