import os

from beamngpy.beamng import BeamNGpy

from beamngpy.logging import config_logging, set_up_simple_logging

from beamngpy.quat import angle_to_quat
from beamngpy.vec3 import vec3

from beamngpy.time_series import Time_Series
from beamngpy.mesh_view import Mesh_View
from beamngpy.us_view import US_View
from beamngpy.trajectory import Trajectory

from beamngpy.visualizer import Visualiser

from beamngpy.scenario import Scenario, ScenarioObject, StaticObject
from beamngpy.scenario.level import Level
from beamngpy.scenario.procedural import ProceduralBump, ProceduralCone, ProceduralCube, ProceduralCylinder, ProceduralMesh, ProceduralRing
from beamngpy.scenario.road import MeshRoad, Road

from beamngpy.vehicle import Vehicle


def read(fil):
    fil = os.path.join(os.path.dirname(__file__), fil)
    with open(fil, encoding='utf-8') as f:
        return f.read()


__version__ = read('version.txt')
