"""
.. module:: sensors
    :platform: Windows
    :synopsis: Module containing the various sensors one can attach to a
               vehicle.
    :noindex:

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
.. moduleauthor:: Sedonas <https://github.com/Sedonas>
.. moduleauthor:: Dave Stark <dstark@beamng.gmbh>

This module implements various sensors that can be attached to vehicles to
extract data from simulations.
"""

from .accelerometer import Accelerometer
from .auto_camera import AutoCamera
from .camera import Camera
from .damage import Damage
from .electrics import Electrics
from .gforces import GForces
from .imu import IMU
from .lidar import Lidar
from .sensor import Sensor
from .state import State
from .timer import Timer
from .ultrasonic import Ultrasonic
