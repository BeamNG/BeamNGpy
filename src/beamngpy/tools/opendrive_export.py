from __future__ import annotations

import math
from datetime import datetime

import numpy as np

from beamngpy import vec3
from beamngpy.tools.navigraph_data import ExportOpenDriveMap

__all__ = ["OpenDriveExporter"]

CACHED_TANGENTS = {}


class OpenDriveExporter:
    """
    A class for exporting BeamNG road network data to OpenDrive (.xodr) format.
    """
    @staticmethod
    def export(name, bng):
        """
        Exports the road network data to OpenDrive (.xodr) format.
        The export contains all road sections, some basic lane data, and some junction connectivity data.

        Args:
            name: The path/filename at which to save the .xodr file.
            bng: The BeamNG instance.
        """
        ExportOpenDriveMap(bng,name)
