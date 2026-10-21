BeamNGpy Reference
******************

BeamNGpy
========

.. autoclass:: beamngpy.BeamNGpy
   :members:
   :undoc-members:

API
---
.. automodule:: beamngpy.api.beamng
   :members:
   :imported-members:
   :undoc-members:
   :show-inheritance:

Vehicle
=======

.. autoclass:: beamngpy.Vehicle
   :members:
   :undoc-members:

Sensors
-------
.. autoclass:: beamngpy.vehicle.Sensors
   :members:

API
---
.. automodule:: beamngpy.api.vehicle
   :members:
   :imported-members:
   :undoc-members:
   :show-inheritance:
   :exclude-members: RootApi

ADAS
----
.. autoclass:: beamngpy.vehicle.lka.LaneKeepingAssist
   :members:
   :undoc-members:

.. autoclass:: beamngpy.vehicle.adas_ultrasonic.AdasUltrasonicApi
   :members:
   :undoc-members:

Scenario
========

.. autoclass:: beamngpy.Scenario
   :members:
   :undoc-members:

.. autoclass:: beamngpy.Level
   :members:
   :undoc-members:

.. autoclass:: beamngpy.ScenarioObject
   :members:
   :undoc-members:

Procedural Objects
------------------

.. autoclass:: beamngpy.ProceduralMesh
   :members:
   :show-inheritance:

.. autoclass:: beamngpy.ProceduralCylinder
   :show-inheritance:

.. autoclass:: beamngpy.ProceduralBump
   :show-inheritance:

.. autoclass:: beamngpy.ProceduralCone
   :show-inheritance:

.. autoclass:: beamngpy.ProceduralCube
   :show-inheritance:

.. autoclass:: beamngpy.ProceduralRing
   :show-inheritance:

Roads
-----

.. autoclass:: beamngpy.Road
   :members:
   :undoc-members:

.. autoclass:: beamngpy.MeshRoad
   :members:
   :undoc-members:

Sensors
=======

Automated Sensors
-----------------

Camera
^^^^^^
.. autoclass:: beamngpy.sensors.Camera
   :members:
   :undoc-members:

Lidar
^^^^^
.. autoclass:: beamngpy.sensors.Lidar
   :members:
   :undoc-members:

Ultrasonic Sensor
^^^^^^^^^^^^^^^^^
.. autoclass:: beamngpy.sensors.Ultrasonic
   :members:
   :undoc-members:

Powertrain Sensor
^^^^^^^^^^^^^^^^^
.. autoclass:: beamngpy.sensors.PowertrainSensor
   :members:
   :undoc-members:

Advanced IMU
^^^^^^^^^^^^
.. autoclass:: beamngpy.sensors.AdvancedIMU
   :members:
   :undoc-members:

Radar
^^^^^
.. autoclass:: beamngpy.sensors.Radar
   :members:
   :undoc-members:

Ideal Radar
^^^^^^^^^^^
.. autoclass:: beamngpy.sensors.IdealRadar
   :members:
   :undoc-members:

Mesh Sensor
^^^^^^^^^^^
.. autoclass:: beamngpy.sensors.Mesh
   :members:
   :undoc-members:

GPS
^^^
.. autoclass:: beamngpy.sensors.GPS
   :members:
   :undoc-members:

Roads Sensor
^^^^^^^^^^^^
.. autoclass:: beamngpy.sensors.RoadsSensor
   :members:
   :undoc-members:

Classical Sensors
-----------------

Sensor
^^^^^^
.. autoclass:: beamngpy.sensors.Sensor
   :members:

State
^^^^^
.. autoclass:: beamngpy.sensors.State
   :show-inheritance:

Electrics
^^^^^^^^^
.. autoclass:: beamngpy.sensors.Electrics
   :show-inheritance:

Timer
^^^^^
.. autoclass:: beamngpy.sensors.Timer
   :show-inheritance:

Damage
^^^^^^
.. autoclass:: beamngpy.sensors.Damage
   :show-inheritance:

GForces
^^^^^^^
.. autoclass:: beamngpy.sensors.GForces
   :show-inheritance:

Logging
=======

.. automodule:: beamngpy.logging
   :members:
   :undoc-members:

Tools
=====

.. autoclass:: beamngpy.tools.OpenDriveExporter
   :members:
   :undoc-members:

.. autoclass:: beamngpy.tools.OpenStreetMapExporter
   :members:
   :undoc-members:

.. autoclass:: beamngpy.tools.SumoExporter
   :members:
   :undoc-members:

.. autoclass:: beamngpy.tools.OpenDriveImporter
   :members:
   :undoc-members:

   Import one of the example networks shipped under ``examples/data``::

      result = OpenDriveImporter(bng).import_xodr("examples/data/Ex_LHT-Complex-X-Junction.xodr")

.. autoclass:: beamngpy.tools.OpenStreetMapImporter
   :members:
   :undoc-members:

   Import one of the example networks shipped under ``examples/data``::

      result = OpenStreetMapImporter(bng).import_osm("examples/data/London.osm")

.. autoclass:: beamngpy.tools.SumoImporter
   :members:
   :undoc-members:

   Pass a common prefix or either paired file path. The importer reads
   ``<prefix>.nod.xml`` and ``<prefix>.edg.xml``, e.g. the shipped
   ``examples/data/London`` pair (small test area, passenger-car edges)::

      result = SumoImporter(bng).import_sumo("examples/data/London.nod.xml")

   The shipped pair was produced from ``examples/data/London.osm`` with
   ``netconvert`` 1.17.0, run in ``examples/data`` (cmd)::

      netconvert --osm-files=London.osm --output-file=_tmp_London.net.xml ^
        --plain-output-prefix=London --proj.utm --geometry.remove ^
        --roundabouts.guess --tls.discard-simple --tls.guess-signals ^
        --ramps.guess --keep-edges.by-vclass=passenger --junctions.join

.. autoclass:: beamngpy.tools.TrafficConfig
   :members:
   :undoc-members:

Template Car Generator
----------------------

.. autoclass:: beamngpy.tools.TemplateCarGenerator
   :members:
   :undoc-members:

.. autopydantic_model:: beamngpy.tools.template_car.Settings
   :members:
   :undoc-members:

.. autopydantic_model:: beamngpy.tools.template_car.TemplateVehicle
   :members:
   :undoc-members:
   :exclude-members: to_inputs

.. autopydantic_model:: beamngpy.tools.template_car.VehicleParameters
   :members:
   :undoc-members:

.. autopydantic_model:: beamngpy.tools.template_car.VehicleStructure
   :members:
   :undoc-members:

.. autopydantic_model:: beamngpy.tools.template_car.Optimization
   :members:
   :undoc-members:

.. autopydantic_model:: beamngpy.tools.template_car.OptimizationVariables
   :members:
   :undoc-members:

.. autopydantic_model:: beamngpy.tools.template_car.TargetValues
   :members:
   :undoc-members:

.. autopydantic_model:: beamngpy.tools.template_car.OptimizationEnabled
   :members:
   :undoc-members:

.. autoclass:: beamngpy.tools.template_car.BodyShape
   :members:
   :undoc-members:

.. autoclass:: beamngpy.tools.template_car.SuspensionFront
   :members:
   :undoc-members:

.. autoclass:: beamngpy.tools.template_car.SuspensionRear
   :members:
   :undoc-members:

Miscellaneous
=============

Colors
------
.. automodule:: beamngpy.misc.colors
   :members:
   :undoc-members:

Quaternions
-----------
.. automodule:: beamngpy.misc.quat
   :members:
   :undoc-members:

Vec3
----
.. automodule:: beamngpy.misc
   :members:
   :imported-members:
   :undoc-members:
   :show-inheritance:

Types
-----

.. automodule:: beamngpy.types
   :members:
   :undoc-members:

Connection
----------

.. automodule:: beamngpy.connection
   :members:
   :undoc-members: