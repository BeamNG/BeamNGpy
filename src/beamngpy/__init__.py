"""
BeamNGPy API module.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
"""

from .beamng import *
from .visualiser import *
from .vehicle import *
from .scenario import *
from .level import *
from . import beamngcommon, sensors
import os


def read(fil):
    fil = os.path.join(os.path.dirname(__file__), fil)
    with open(fil, encoding='utf-8') as f:
        return f.read()


__version__ = read('version.txt')
