
BeamNG MATLAB integration
*************************


Overview
^^^^^^^^

We are excited to announce that the highly requested feature of bridging BeamNG.tech and `MATLAB <https://www.mathworks.com/products/matlab.html>`_ is here. MATLAB, with its long history as an academic engineering and mathematical tool, is a programming and numeric computing platform used to analyse data, develop algorithms, and create models. The newly created bridge with MATLAB will enable you to run, control, and interact with the BeamNG.tech simulation. We have integrated five main scripts for your convenience in making use of annotations, bounding boxes, multi-shot camera, object placement, vehicle state plotting and creation of simple scenarios on our East Coast USA map.

Prequest
^^^^^^^^

you must have the following softwares/packages installed:

1.  `Compatible Python <https://www.mathworks.com/support/requirements/python-compatibility.html>`_

2. `BeamNGpy <https://pypi.org/project/beamngpy/>`__

3. `BeamNG.Tech <https://documentation.beamng.com/beamng_tech/>`_


The `BeamNG-MATLAB-integration bridge <https://github.com/BeamNG/BeamNG-MATLAB-integration>`_ is depending on `BeamNG.Tech <https://documentation.beamng.com/beamng_tech/>`_ and `BeamNGpy <https://documentation.beamng.com/beamng_tech/beamngpy/>`__. Make sure that you have the license for BeamNG.Tech. The Github repository of the BeamNG-MATLAB has some basic examples of scripts that run a vehicle with some sensors ex. Lidar, Camera, and state sensor.

Compatibility
=============


Running the BeamNG ROS integration requires three individual software components, here is a list of compatible versions.


+-------------+----------+---------------------------+--------+----------+
| BeamNG.tech | BeamNGpy | BeamNG MATLAB integration | MATLAB | Python   |
+=============+==========+===========================+========+==========+
| 0.28        | 1.26     | 0.1.1                     | R2023a | 3.9      |
+-------------+----------+---------------------------+--------+----------+
| 0.27        | 1.25.1   | 0.1.0                     | R2022b | 3.9      |
+-------------+----------+---------------------------+--------+----------+



1. Setup a compatible python version
====================================

After installing the `compatible python version <https://www.mathworks.com/support/requirements/python-compatibility.html>`_ with MATLAB, make sure to include the path of excutable python file (exe) in your in "path" variable of "environment variables" as explained `here <https://docs.oracle.com/en/database/oracle/machine-learning/oml4r/1.5.1/oread/creating-and-modifying-environment-variables-on-windows.html#GUID-DD6F9982-60D5-48F6-8270-A27EC53807D0>`_.

2. Run python engine in MATLAB
==============================

Run the `test_python.m <https://github.com/BeamNG/BeamNG-MATLAB-integration/blob/main/test_python.m>`_ to make sure that python engine is connected to your MATLAB engine as shown in the picture below.


.. image:: https://raw.githubusercontent.com/BeamNG/BeamNG-MATLAB-integration/main/media/test_python.png
  :width: 800
  :alt: Testing python in MATLAB




Vehicle State Plotting
^^^^^^^^^^^^^^^^^^^^^^

Use the state sensor to plot some graphs of the vehicle position, wheel speed and direction, throttle, and brake.

.. image:: https://raw.githubusercontent.com/BeamNG/BeamNG-MATLAB-integration/main/media/annotation_bounding_boxes.png
  :width: 800
  :alt: Vehicle state ploting


Running Lidar sensor, and AI control.
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Create a simple scenario
2. Use the simulator's AI with BeamNGpy

.. image:: https://raw.githubusercontent.com/BeamNG/BeamNG-MATLAB-integration/main/media/lidar_tour.png
  :width: 800
  :alt: Lidar sensor and AI control mode

Multi-shot Camera
^^^^^^^^^^^^^^^^^

Change the position and rotation of a camera

.. image:: https://raw.githubusercontent.com/BeamNG/BeamNG-MATLAB-integration/main/media/multi_shots_1.png
  :width: 800
  :alt: Multi-shot Camera


Object Placement
^^^^^^^^^^^^^^^^

1. Define a custom scenario for a given map
2. Generate procedural content, i.e. simple meshes

.. image:: https://raw.githubusercontent.com/BeamNG/BeamNG-MATLAB-integration/main/media/object_placment_0.png
  :width: 800
  :alt: Object Placement



Annotation and Bounding Boxes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Get semantic annotations
2. Get instance annotations
3. Graw bounding boxes (note that this feature is not ready for use yet)

.. image:: https://raw.githubusercontent.com/BeamNG/BeamNG-MATLAB-integration/main/media/annotation_bounding_boxes.png
  :width: 800
  :alt: Annotation and Bounding Boxes
