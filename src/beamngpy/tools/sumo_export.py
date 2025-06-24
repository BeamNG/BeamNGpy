from __future__ import annotations

from beamngpy.tools.navigraph_data import ExportSumo

__all__ = ["SumoExporter"]



class SumoExporter:

    @staticmethod
    def export(name, bng):
        """
        Exports the road network data to Sumo (.nod.xml and .edg.xml) format.
        The export contains all road sections, some basic lane data, and some junction connectivity data.
        This function will generate both .xml files required to generate the Sumo road network. The user should then type the following into the command prompt:
        `netconvert --node-files=<NAME>.nod.xml --edge-files=<NAME>.edg.xml -o converted.net.xml`
        which will then generate the final road network, which can be loaded with the sumo applications.

        Args:
            name: the filename prefix, by which to save the sumo road network (the .nod.xml and .edg.xml extensions will be appended to the end of this name).
            bng: The BeamNG instance.
        """
        ExportSumo(bng,name)