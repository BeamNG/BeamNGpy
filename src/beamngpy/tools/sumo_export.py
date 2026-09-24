from __future__ import annotations

from beamngpy.tools.navigraph_data import ExportSumo

__all__ = ["SumoExporter"]



class SumoExporter:

    @staticmethod
    def export(name, bng):
        """
        Exports the road network data to SUMO (.nod.xml and .edg.xml) format.

        The export contains road sections, lane counts, speed limits, one-way
        direction, and junction connectivity from the loaded map's navgraph.
        Files are written next to the current BeamNG.tech user folder (typically
        ``<userpath>/BeamNG.tech/current/``).

        After export, convert and smoke-test the network in SUMO (cmd)::

            netconvert --node-files=<NAME>.nod.xml --edge-files=<NAME>.edg.xml -o <NAME>.net.xml
            
            python "%SUMO_HOME%\\tools\\randomTrips.py" -n <NAME>.net.xml -r <NAME>.rou.xml -e 3600 -p 2
            
            sumo-gui -n <NAME>.net.xml -r <NAME>.rou.xml

        ``netconvert`` may print warnings and still succeed. Wait until
        ``randomTrips.py`` finishes writing ``<NAME>.rou.xml`` before launching
        ``sumo-gui``. This checks that the exported network loads and can carry

        Args:
            name: Filename prefix for the SUMO files (``.nod.xml`` and
                ``.edg.xml`` are appended).
            bng: The BeamNG instance.
        """
        ExportSumo(bng,name)