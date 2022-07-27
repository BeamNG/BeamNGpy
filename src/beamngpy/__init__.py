"""
BeamNGPy API module.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
"""

import os

from . import beamngcommon, sensors
from .beamng import *
from .level import *
from .procedural import *
from .road import *
from .scenario import *
from .vehicle import *
from .visualiser import *


def read(fil):
    fil = os.path.join(os.path.dirname(__file__), fil)
    with open(fil, encoding='utf-8') as f:
        return f.read()


__version__ = read('version.txt')
