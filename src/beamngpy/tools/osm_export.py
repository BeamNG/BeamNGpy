from __future__ import annotations

from datetime import datetime

from beamngpy import vec3
from beamngpy.tools.navigraph_data import ExportOpenStreetMap

__all__ = ["OpenStreetMapExporter"]




class OpenStreetMapExporter:
    """
    A class for exporting BeamNG road network data to OpenStreetMap (.osm) format.
    """
    @staticmethod
    def export(name, bng):
        """
        Exports the road network data to OpenStreetMap (.osm) format.
        The export contains all road sections, some basic lane data, and some junction connectivity data.

        Args:
            name: The path/filename at which to save the .osm file.
            bng: The BeamNG instance.
        """
        ExportOpenStreetMap(bng,name)
