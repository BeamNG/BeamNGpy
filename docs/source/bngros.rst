BeamNG ROS Integration
**********************

To support the interoperability between BeamNG.tech and ROS we published the BeamNG ROS Integration.
It is an independent ROS package that translates a range of BeamNGpy features to the ROS framework. `beamng-ros-integration <https://github.com/BeamNG/beamng-ros-integration>`_ is an repository contains packages to support the interoperability between BeamNG.tech and ROS 1 distributions Melodic Morenia and Noetic Ninjemys.

Basic ROS functionality are included i.e., sensors streaming, `Rviz <http://wiki.ros.org/rviz>`_ simulation, direct keyboard control (Teleop). ROS topics for Sensor Suite:  multiple filters of the camera (Annotated, instance, Depth, and RGB), 3D Lidar, Ultrasonic, IMU, and vehicle electrics (speed, fuel, temperature, gear, signals, lights, etc).


- Installation Prerequisites:

you must have the following softwares/packages installed i.e., `BeamNG.Tech <https://documentation.beamng.com/beamng_tech/>`_, `BeamNGpy <https://pypi.org/project/beamngpy/>`__,and `WSL2 <https://jack-kawell.com/2020/06/12/ros-wsl2/>`_.


ROS packages
^^^^^^^^^^^^

- beamng_agent: for the control of a driving agent used for Teloep movement of the beamng_teleop_keyboard package, also used for enable/disable keyboard remote control to the BeamNG.Tech simulation platform.

- beamng_control: loading the ROS-BeamNG.Tech bridge and the scenario details (vehicle, environment, sensors, location, etc.).

- beamng_msgs: Defind the custom messages of the BeamNG.Tech simulator to be readable by ROS-standards.

- beamng_teleop_keyboard: keyboard remote control of the BeamNG.Tech simulation platform through ROS bridge.

Compatibility
^^^^^^^^^^^^^

Running the BeamNG ROS integration requires three individual software components, here is a list of compatible versions.

+-------------+----------+------------------------+
| BeamNG.tech | BeamNGpy | BeamNG ROS Integration |
+=============+==========+========================+
| 0.30        | 1.26.1   | 0.1.4                  |
+-------------+----------+------------------------+
| 0.28        | 1.26     | 0.1.3                  |
+-------------+----------+------------------------+
| 0.27        | 1.25.1   | 0.1.2                  |
+-------------+----------+------------------------+
| 0.26        | 1.24     | 0.1.1                  |
+-------------+----------+------------------------+
| 0.25        | 1.23.1   | 0.1.0                  |
+-------------+----------+------------------------+

WSL2 setup
^^^^^^^^^^
ROS1 integration is built on top of Windows Subsystem for Linux (WSL2). The recommended linux version is `ubuntu focal 20.04 <http://old-releases.ubuntu.com/releases/focal/>`_, and The recommended ROS 1 distribution is `Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_. The ROS bridge is made through python API support from `BeamNGpy <https://github.com/BeamNG/BeamNGpy>`_.


ROS setup
^^^^^^^^^
setup `catkin_ws <http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>`_

Install and build ROS bridge

``git@github.com:BeamNG/beamng-ros-integration.git``

``cd ~/catkin_ws/ && catkin_make``

WSL2 dependencies:

``sudo apt install python3-rosdep2``

``sudo apt install python3-pip``

``pip install beamngpy``

``sudo apt-get install ros-noetic-rostest``

``sudo apt-get install ros-noetic-actionlib``

``python3 -m pip install -U scikit-image``

``sudo apt install python3-rosservice``


Getting started
^^^^^^^^^^^^^^^

BeamNG-ROS bridge needs to be configured to contain the correct IPv4 address of the machine hosting the simulation.
Using it will start up a node that connects to the simulation and starts up a scenario as defined in the `beamng_control/config/scenario/{scenario}.json`. Other scenario specifications are available in the same directory.

- Scenarios are defined through JSON objects, here is a list of possible keys and values.


+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|Key                   |Value Type        | Value Specification                                                                 | Entry Type |
+======================+==================+=====================================================================================+============+
|``version``           |String            | BeamnG ROS Integration version, f.ex. ``1``                                         | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``level``             |String            | BeamNG.tech level name, f.ex. ``west_coast_usa``                                    | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``mode``              |String            | Value                                                                               | Optional   |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``vehicles``          |Array             | At least one vehicle needs to be specified in order to obtain a valid scenario.     | Mandatory  |
|                      |                  | See the table below for the Specification.                                          |            |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``name``              |String            | Name of the level.                                                                  | Mandatory  |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``time_of_day``       |Float             | Value between ``0`` and ``1`` where the range ``[0, .5]`` corresponds               | Optional   |
|                      |                  | to the times between 12 a.m. and 12 p.m. and ``[.5], 1]`` corresponds to            |            |
|                      |                  | the time range between 12 p.m. and 12 a.m.                                          |            |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+
|``weather_presets``   |String            | Weather presets are level specific, **ToDo**                                        | Optional   |
+----------------------+------------------+-------------------------------------------------------------------------------------+------------+




- Vehicles are also defined as JSON objectsin `beamng_control/config/vehicles/{vehicle}.json`.

+-----------------------------+------------------+------------------------------------------------------------------------------------------------------------------------+------------+
|Key                          |Value Type        | Value Specification                                                                                                    | Entry Type |
+=============================+==================+========================================================================================================================+============+
|``name``                     |String            |Name of the vehicle, used for identification                                                                            | Mandatory  |
+-----------------------------+------------------+------------------------------------------------------------------------------------------------------------------------+------------+
|``model``                    |String            |Name of the vehicle type, f.ex. ``etk800``                                                                              | Mandatory  |
+-----------------------------+------------------+------------------------------------------------------------------------------------------------------------------------+------------+
|``position``                 |Array             |Array of 3 floats, specifying the ``x``, ``y``, and ``x`` position of the vehicle.                                      | Mandatory  |
+-----------------------------+------------------+------------------------------------------------------------------------------------------------------------------------+------------+
|``rotation``                 |Array             |Array of 4 floats, specifying the vehicle rotation quaternion.                                                          | Mandatory  |
+-----------------------------+------------------+------------------------------------------------------------------------------------------------------------------------+------------+
|``sensors_classical``        |Array             |Array of JSON objects, specifying the vehicles sensor parameters i.e., electrics, IMU, damage, gforce, and time sensor  | Optional   |
+-----------------------------+------------------+------------------------------------------------------------------------------------------------------------------------+------------+
|``sensors_automation``       |Array             |Array of JSON objects, specifying the ad-hoc_sensors parameters i.e., Lidar, camera, and Ultrasonic sensor              | Optional   |
+-----------------------------+------------------+------------------------------------------------------------------------------------------------------------------------+------------+


Running BeamNG.Tech
^^^^^^^^^^^^^^^^^^^

After installing `BeamNGpy <https://github.com/BeamNG/BeamNGpy>`__, and setup BeamNG.Tech, you can run BeamNG.py from the Powershell as shown in the picture below.

.. image:: https://github.com/BeamNG/BeamNGpy/raw/master/media/bngpy.png
  :width: 800
  :alt: Run BeamNG.Tech from BeamNGpy
.. ![Run BeamNG.Tech from BeamNGpy](https://github.com/BeamNG/BeamNGpy/raw/master/media/bngpy.png)


Running the ROS-bridge
^^^^^^^^^^^^^^^^^^^^^^
* Loading beamng_control node for loading the scenarios components i.e., level, vehicle, environemnt and sensors from `example.launch` file in the `beamng_control` package through the command:

``roslaunch beamng_control example.launch``

Running beamng_agent
^^^^^^^^^^^^^^^^^^^^
* Loading beamng_agent node for enabling the control from ROS side:
    ``roslaunch beamng_agent example.launch``

The folloing topics for move/stop the vehicle in simulation and enable/disable keybard control from the simulation side; using an array of commands as following [steering throttle brake parkingbrake clutch gear], here's some exmaples of the ```VehicleControl``` :

* Driving:
    ``rostopic pub --once control beamng_msgs/VehicleControl 0 1 0 0 0 1``


- Stopping:
    ``rostopic pub --once control beamng_msgs/VehicleControl 0 0 1 0 0 1``


* Release:
    ``rostopic pub --once control beamng_msgs/VehicleControl 0 0 0 0 0 1``


Calling ROS-services for controlling the Simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To Dis-/Enables user keyboard and mouse control of the BeamNG.Tech simulator

+--------------------------------------+-----------------------------------------------+-----------------------------+
|Name                                  |  Type                                         |  Purpose                    |
+======================================+==================+============================+=============================+
|``/beamng_control/pause``             | ``bng_msgs.srv.ChangeSmulationState``         |  Pause the simulation.      |
+--------------------------------------+-----------------------------------------------+-----------------------------+
|``/beamng_control/resume``            | ``bng_msgs.srv.ChangeSmulationState``         |  Resume the simulation.     |
+--------------------------------------+-----------------------------------------------+-----------------------------+


- Disable user keyboard and mouse control of the BeamNG.Tech simulator:
  ``rosservice call /beamng_control/pause "{}"``
- terminal feedback should be:

  ``success: True``

- Enable user keyboard and mouse control of the BeamNG.Tech simulator:
  ``rosservice call /beamng_control/resume "{}"``

- terminal feedback should be:

  ``success: True``





Vehicle Creation and Control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Various services to control the state of the simulation are available.

+---------------------------------------------+---------------------------------------------+------------------------------------------------------+
|Name                                         |Type                                         | Purpose                                              |
+=============================================+=============================================+======================================================+
|``/beamng_control/get_scenario_state``       |``bng_msgs.srv.GetScenarioState``            | Determining the current state of thescenario.        |
+---------------------------------------------+---------------------------------------------+------------------------------------------------------+
|``/beamng_control/spawn_vehicle``            |``beamng_msgs.srv.SpawnVehicle``             | Spawn new vehicle.                                   |
+---------------------------------------------+---------------------------------------------+------------------------------------------------------+
|``/beamng_control/teleport_vehicle``         |``beamng_msgs.srv.TeleportVehicle``          | Teleport vehicle.                                    |
+---------------------------------------------+---------------------------------------------+------------------------------------------------------+
|``/beamng_control/start_scenario``           |``bng_msgs.srv.StartScenario``               | Starting a loaded scenario.                          |
+---------------------------------------------+---------------------------------------------+------------------------------------------------------+
|``/beamng_control/get_current_vehicles``     |``beamng_msgs.srv.GetCurrentVehiclesInfo``   | Get the spawned vehicle information.                 |
+---------------------------------------------+---------------------------------------------+------------------------------------------------------+


- Clone a new vehicle:
    ``rosservice call /beamng_control/spawn_vehicle 'ros' [0,5,10] [0,0,0,1] "/config/vehicles/etk800.json"``


- Load a new scenario:
    ``rosservice call /beamng_control/start_scenario "/config/scenarios/west_coast.json"``


- Reposition the current vehicle in west coast:
    ``rosservice call /beamng_control/teleport_vehicle "ego_vehicle" [568.908,13.422,148.565] [0,0,0,1]``


- Getting the scenario state:
    ``rosservice call /beamng_control/get_scenario_state``


- Getting the get_current_vehicles:
    ``rosservice call /beamng_control/get_current_vehicles``


- Getting the get_loggers:
   ``rosservice call /beamng_control/get_loggers``

Note
^^^^

  - if you got a feedback `success: False` for `resume` or `pause` services, that means your `beamng_agent` node isn't active, and you will getting the following error message in the terminal of `beamng_control` node:






List of ROS-topics
^^^^^^^^^^^^^^^^^^

* ROS-visualization tool (Rviz) map:
      ``/beamng_control/<vehicle_id>/marker``


.. .. image:: https://github.com/BeamNG/BeamNGpy/raw/master/media/rviz_west_coast_usa.png
..   :width: 800
..   :alt: Rviz Map of road network West Coast, US

.. ![Rviz Map of road network West Coast, USA](https://github.com/BeamNG/BeamNGpy/raw/master/media/rviz_west_coast_usa.png)


* Camera:

Contrary to other sensors, the Camera sensor may publish to multiple topics.
If the camera sensor is configured to collect color, depth, annotation, and instance data, it is published to the respective topics:

      ``/beamng_control/<vehicle_id>/<camera_id>/color``

      ``/beamng_control/<vehicle_id>/<camera_id>/depth``

      ``/beamng_control/<vehicle_id>/<camera_id>/annotation``

      ``/beamng_control/<vehicle_id>/<camera_id>/instance``

The message type for all topics is `sensor_msgs.msg.Image`.
Note that although the bounding_box option is given, this feature is still under development and will automatically be disabled.

+--------------------+------------------+---------------------------------------------------------------------------------------+------------+
|Key                 |Value Type        | Value Specification                                                                   | Entry Type |
+====================+==================+=======================================================================================+============+
|``type``            | String           | ``Camera.default``                                                                    | Mandatory  |
+--------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``name``            | String           | Unique sensor id.                                                                     | Mandatory  |
+--------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``position``        | Array            | Array of 3 floats, specifying the ``x``, ``y``, and ``x`` position of the sensor.     | Mandatory  |
+--------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``orientation``     | Array            | Array of 4 floats, specifying the vehicle rotation quaternion                         | Mandatory  |
+--------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``resolution``      | Array            | Tuple of ints, defining the ``x`` and ``y`` resolution of                             | Optional   |
|                    |                  | the resulting images.                                                                 |            |
+--------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``fov``             | Integer          | Camera field of view.                                                                 | Optional   |
+--------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``colour``          | Boolean          | Dis-/Enables color image generation.                                                  | Optional   |
+--------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``depth``           | Boolean          | Dis-/Enables depth image generation.                                                  | Optional   |
+--------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``annotation``      | Boolean          | Dis-/Enables ground truth generation for object type annotation.                      | Optional   |
+--------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``instance``        | Boolean          | Dis-/Enables ground truth generation for instance annotation.                         | Optional   |
+--------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``bounding_box``    | Boolean          | This feature is not supoprted at the moment                                           | Optional   |
|                    |                  | and will be **automatically disabled**.                                               |            |
+--------------------+------------------+---------------------------------------------------------------------------------------+------------+

.. .. image:: https://github.com/BeamNG/BeamNGpy/raw/master/media/rqt_camera.png
..   :width: 800
..   :alt: multiple camera filters rgb,depth,insthence,and annotation -starting from top-left to bottom-right

.. ![multiple camera filters rgb,depth,insthence,and annotation -starting from top-left to bottom-right](https://github.com/BeamNG/BeamNGpy/raw/master/media/rqt_camera.png)


* LiDAR:

Message type: `sensor_msgs.msg.PointCloud2`
    ``/beamng_control/<vehicle_id>/<lidar_id>``

+-----------------------------------+------------------+---------------------------------------------------------------------------------------+------------+
|Key                                |Value Type        | Value Specification                                                                   | Entry Type |
+===================================+==================+=======================================================================================+============+
|``type``                           | String           | ``Lidar.default``                                                                     | Mandatory  |
+-----------------------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``name``                           | String           | Unique sensor id.                                                                     | Mandatory  |
+-----------------------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``position``                       | Array            | Array of 3 floats, specifying the ``x``, ``y``, and ``x`` position of the sensor.     | Mandatory  |
+-----------------------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``rotation``                       | Array            | Array of 3 floats, specifying the vehicle rotation quaternion                         | Mandatory  |
+-----------------------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``vertical_resolution``            | Integer          | Vertical resolution, i.e. how many lines are sampled vertically                       | Optional   |
+-----------------------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``vertical_angle``                 | Float            | The vertical LiDAR sensor angle, in degrees.                                          | Optional   |
+-----------------------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``frequency``                      | Integer          | The frequency of this LiDAR sensor.                                                   | Optional   |
+-----------------------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``rays_per_second``                | Integer          | The rays per second emmited by the LiDAR sensor                                       | Optional   |
+-----------------------------------+------------------+---------------------------------------------------------------------------------------+------------+
|``is_visualised``                  | Boolean          | Dis-/Enable in-simulation visualization.                                              | Optional   |
+-----------------------------------+------------------+---------------------------------------------------------------------------------------+------------+

.. .. image:: https://github.com/BeamNG/BeamNGpy/raw/master/media/lidar_west_coast_usa.png
..   :width: 800
..   :alt: 3D-LiDAR sensor reading
.. ![3D-LiDAR sensor reading](https://github.com/BeamNG/BeamNGpy/raw/master/media/lidar_west_coast_usa.png)
.. ! image:: https://github.com/BeamNG/BeamNGpy/raw/master/media/lidar_west_coast_usa.png


* Ultrasonic sensor :

Message type: `sensor_msgs.msg.Range`
    ``/beamng_control/<vehicle_id>/<ultrasonic_sensor_name>``

+----------------------------+------------------+-------------------------------------------------------------------------------------------------+------------+
|Key                         |Value Type        | Value Specification                                                                             | Entry Type |
+============================+==================+=================================================================================================+============+
|``type``                    | String           | ``Ultrasonic.smallrange``,and/or  ``Ultrasonic.midrange``,and/or  ``Ultrasonic.largerange``     | Mandatory  |
+----------------------------+------------------+-------------------------------------------------------------------------------------------------+------------+
|``name``                    | String           | Unique sensor id.                                                                               | Mandatory  |
+----------------------------+------------------+-------------------------------------------------------------------------------------------------+------------+
|``position``                | Array            | Array of 3 floats, specifying the ``x``, ``y``, and ``x`` position of the sensor.               | Mandatory  |
+----------------------------+------------------+-------------------------------------------------------------------------------------------------+------------+
|``rotation``                | Array            | Array of 3 floats, specifying the vehicle rotation quaternion                                   | Mandatory  |
+----------------------------+------------------+-------------------------------------------------------------------------------------------------+------------+
|``is_visualised``           | Boolean          | Dis-/Enable in-simulation visualization.                                                        | Optional   |
+----------------------------+------------------+-------------------------------------------------------------------------------------------------+------------+

* Damage:

Message type: `beamng_msgs.msg.DamagSensor`
    ``/beamng_control/<vehicle_id>/<damage_sensor_id>``

+--------------------+------------------+------------------------------------------------------------------------+------------+
|Key                 |Value Type        | Value Specification                                                    | Entry Type |
+====================+==================+========================================================================+============+
|``type``            | String           | ``Damage``                                                             | Mandatory  |
+--------------------+------------------+------------------------------------------------------------------------+------------+
|``name``            | String           | Unique sensor id.                                                      | Mandatory  |
+--------------------+------------------+------------------------------------------------------------------------+------------+


.. image:: https://github.com/BeamNG/BeamNGpy/raw/master/media/lidar_camera_and_road.png
  :width: 800
  :alt: Vehicle-Damage reading

.. ![Vehicle-Damage reading](https://github.com/BeamNG/BeamNGpy/raw/master/media/damage_west_coast_usa.png)




* time:

Message type: `beamng_msgs.msg.TimeSensor`
    ``/beamng_control/<vehicle_id>/<time_sensor_id>``

+--------------------+------------------+------------------------------------------------------------------------+------------+
|Key                 |Value Type        | Value Specification                                                    | Entry Type |
+====================+==================+========================================================================+============+
|``type``            | String           | ``Timer``                                                              | Mandatory  |
+--------------------+------------------+------------------------------------------------------------------------+------------+
|``name``            | String           | Unique sensor id.                                                      | Mandatory  |
+--------------------+------------------+------------------------------------------------------------------------+------------+


* Gforces:

Message type: `beamng_msgs.msg.GForceSensor`
    ``/beamng_control/<vehicle_id>/<gforce_sensor_id>``

+--------------------+------------------+------------------------------------------------------------------------+------------+
|Key                 |Value Type        | Value Specification                                                    | Entry Type |
+====================+==================+========================================================================+============+
|``type``            | String           | ``GForces``                                                            | Mandatory  |
+--------------------+------------------+------------------------------------------------------------------------+------------+
|``name``            | String           | Unique sensor id.                                                      | Mandatory  |
+--------------------+------------------+------------------------------------------------------------------------+------------+


* Electrics:

Message type: `beamng_msgs.msg.ElectricsSensor`
    ``/beamng_control/<vehicle_id>/<electrics_sensor_id>``

+----------------------+------------------+------------------------------------------------------------------------+------------+
|Key                   |Value Type        | Value Specification                                                    | Entry Type |
+======================+==================+========================================================================+============+
|``type``              | String           | ``Electrics``                                                          | Mandatory  |
+----------------------+------------------+------------------------------------------------------------------------+------------+
|``name``              | String           | Unique sensor id.                                                      | Mandatory  |
+----------------------+------------------+------------------------------------------------------------------------+------------+

* Imu pose:

Message type: `sensor_msgs.msg.Imu`
    ``/beamng_control/<vehicle_id>/<imu_sensor_id>``

+--------------------+------------------+----------------------------------------------------------------------------------+------------+
|Key                 |Value Type        | Value Specification                                                              | Entry Type |
+====================+==================+==================================================================================+============+
|``type``            | String           | ``IMU``                                                                          | Mandatory  |
+--------------------+------------------+----------------------------------------------------------------------------------+------------+
|``name``            | String           | Unique sensor id.                                                                | Mandatory  |
+--------------------+------------------+----------------------------------------------------------------------------------+------------+
|``position``        | Array            | Array of 3 floats, specifying the ``x``, ``y``, and ``x`` position of the sensor.| Mandatory  |
+--------------------+------------------+----------------------------------------------------------------------------------+------------+

.. image:: https://github.com/BeamNG/BeamNGpy/raw/master/media/imu_west_coast_usa.png
  :width: 800
  :alt: IMU sensor reading
.. ![IMU sensor reading](https://github.com/BeamNG/BeamNGpy/raw/master/media/imu_west_coast_usa.png)


* Vehicle state:

Message type: `beamng_msgs.msg.StateSensor`
    ``/beamng_control/<vehicle_id>/state``



Teleop_control
^^^^^^^^^^^^^^


`beamng_teleop_keyboard <https://github.com/BeamNG/beamng-ros-integration/tree/master/beamng_teleop_keyboard>`_ is a generic Keyboard Packages is built for teleoperating ROS robots using Twist message from `geometry_messages <https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html>`_.

Running beamng_teleop_keyboard
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Loading BeamNG-ROS bridge:
    ``roslaunch beamng_control example.launch``

- Calling Twist_message converter node:
    ``rosrun beamng_teleop_keyboard converter``

- Calling Teleop node:
    ``rosrun beamng_teleop_keyboard teleop_key``

- Loading beamng_agent node:
    ``roslaunch beamng_agent example.launch``


