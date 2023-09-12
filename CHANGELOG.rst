=========
Changelog
=========

Version 1.26.1
==============

- New features

  - OpenDrive (.xodr) importer added, and new example created in Examples folder.

  - OpenStreetMap (.osm) importer and exporter added, and new examples created in Examples folder.

  - Eclipse Sumo (.nod.xml and .edg.xml) importer and exporter added, and new examples created in Examples folder.

- BeamNGpy fixes / improvements

  - Improved/added documentation

    - ``Scenario`` class now has all parameters documented.
    - ``BeamNGpy.debug`` API methods are now documented
    - ``BeamNGpy.env`` now contains more information about the 'time of day' object
    - Added documentation for RADAR and Mesh sensors

  - ``Vehicle.set_part_config`` now does not recreate the existing connection to the simulator, as it was not needed

  - Small refactor of unit tests, the automated sensor scripts are now also runnable under the ``pytest`` framework

  - Invalid vehicle and scene object names produced error in the simulation, now the validation is done on BeamNGpy side

    - name cannot start with the ``%`` character or a digit
    - name cannot contain the ``/`` character
  - Added new options to ``BeamNGpy.scenario.load`` called ``connect_player_vehicle`` and ``connect_existing_vehicles``

    - ``connect_player_vehicle`` is ``True`` by default and it connects the player vehicle to the simulation after scenario load
    - ``connect_existing_vehicles`` is ``True`` by default and it connects all the already existing vehicles to the simulation after scenario load
    - setting these options to ``False`` can reduce the loading time by skipping the connection-establishing part, and these vehicles can still be connected manually using ``Vehicle.connect``

  - Added ``crash_lua_on_error`` option to the BeamNGpy constructor

    - behaves in the same way as the option of the same name in ``BeamNGpy.open``


Version 1.26
============
- RADAR sensor

  - Sensor currently works with static scenery but not vehicles.  Will be added in later update.
  - Sensor comes with standard Lua API and BeamNGpy API.
  - Example scripts `provided <https://github.com/BeamNG/BeamNGpy/blob/master/examples/radar_analysis.ipynb>`__ in BeamNGpy.
- Vehicle meshes now available in BeamNGpy

  - Can provide data up to 2000 times per second.
  - Vehicle nodes and physics triangle data available in BeamNGpy, including for individual vehicle wheels.
  - Comes with standard Lua API and BeamNGpy API.
  - Post-processing written in BeamNGpy to compute mesh connectivity data and analyse the mesh data (position, mass, force, velocity).
  - Example scripts `provided <https://github.com/BeamNG/BeamNGpy/blob/master/examples/vehicle_mesh_data.py>`__ in BeamNGpy.
- IMU sensor

  - Added ability to filter gyroscopic readings (as well as acceleration readings). Separate data filtering is used for each.
- Sensor suite bug fixes

  - Fix: problem when changing the requested update times/priority parameters after various sensors were already created, sensor would not update correctly/quickly.
  - Fix: gravity vector was not being applied correctly in IMU sensor.
  - Fix: camera images from static sensors were being rendered upside down.
  - Fix: LiDAR sensor was not returning the whole point cloud in BeamNGpy
- Export BeamNG maps as .xodr files (OpenDrive)

  - BeamNGpy now provides the option to export our map road networks as .xodr files (OpenDrive). The exported road networks contain elevation and road wideness data, along with junction connectivity. On top of this, BeamNGpy also includes a new `class <https://beamngpy.readthedocs.io/en/latest/beamngpy.html#beamngpy.tools.RoadNetworkExporter>`_ with which to analyse the road network data oneself, and process it as required.
- BeamNGpy fixes / improvements

  - Optimized the speed of depth camera processing
  - Added new API:

    - ``BeamNGpy.env.get_tod`` for getting the information about the time of day
    - ``BeamNGpy.env.set_tod`` for setting the time-of-day information, allowing to control the day/night cycle from Python
    - ``BeamNGpy.env.get_gravity`` for getting the current value of the strength of gravity in the simulator.
    - ``Vehicle.get_center_of_gravity`` for getting the center of gravity of a vehicle.

  - Added option to remove procedural meshes
  - Added new option to ``BeamNGpy.open`` called ``crash_lua_on_error``

    - If ``False`` (the default), then Lua crashes in the simulator will not break the connection between BeamNG.tech and BeamNGpy. Set to ``True`` for getting proper stacktraces and easier debugging.
  - Added new option to ``BeamNGpy.scenario.load`` called ``precompile_shaders``

    - If ``True`` (the default), asynchronous shader compilation is disabled. That means the first loading of a map will take longer time, but all parts of the map will be preloaded. If ``False``, the camera sensor can have issues shortly after starting the scenario.
  - Better handling of errors and crashes in the BeamNGpy TCP protocol.
  - Fixed ``vehicle.control`` with zero integer arguments being ignored.
  - Re-added ``BeamNGpy.scenario.get_vehicle`` (removed by accident in the last release).
  - ``BeamNGpy.settings.set_deterministic`` and ``BeamNGpy.settings.set_steps_per_second`` are not persistent anymore and are applied only for a single run of the simulation.

Version 1.25.1
==============
- fixed in BeamNG.tech v0.27.1.0: converted all vehicle rotations sent to BeamNGpy to be consistent with each other
  - if the rotation you are using is 180° rotated across the Y axis, you can use the ``beamngpy.quat.flip_y_axis`` function to flip it
- fixed ``BeamNGpy.vehicles.replace`` to respect vehicle color and license plate text

Version 1.25
============
- Added type hints to the whole BeamNGpy codebase
- Updated `documentation <https://beamngpy.readthedocs.io/en/latest/>`_ to be more readable

- Modularized BeamNGpy API

  - The functions on the BeamNGpy object are now split into modules for easier navigation:

    - ``BeamNGpy.camera`` - configuring the in-game camera
    - ``BeamNGpy.control`` - controlling the simulator state (pausing, stepping, quitting the simulator)
    - ``BeamNGpy.debug`` - drawing debug objects
    - ``BeamNGpy.env`` - controlling the environment state (time of day, gravity)
    - ``BeamNGpy.scenario`` - loading/starting/stopping a BeamNG scenario
    - ``BeamNGpy.settings`` - changing the simulator's settings
    - ``BeamNGpy.system`` - info about the host system
    - ``BeamNGpy.traffic`` - controlling the traffic
    - ``BeamNGpy.ui`` - controlling the GUI elements of the simulator
    - ``BeamNGpy.vehicles`` - controlling vehicles
  - Some of the functions on the ``Vehicle`` object are also moved into modules for easier navigation:

    - ``Vehicle.ai`` - controlling the AI of the vehicle
    - ``Vehicle.logging`` - controlling the in-game logging
  - the previous, not modularized API is still available for backwards compatibility reasons
  - see more in the `documentation <https://beamngpy.readthedocs.io/en/latest/>`_

- Advanced IMU sensor

  - replaces the accelerometer sensor from last release
  - improves upon the existing IMU sensor by using a more advanced algorithm, and provides readings at up to 2000 Hz
- Powertrain sensor

  - new sensor for analysing powertrain properties at high frequency (up to 2000 Hz)
  - new test/demo scripts are available to show execution of this sensor

- New BeamNGpy functionality

  - added support for a custom binary name in BeamNGpy constructor
  - ``BeamNGpy.traffic.spawn`` to spawn traffic without a set of predefined vehicles
  - ``BeamNGpy.traffic.reset`` to reset all traffic vehicles from the player (teleport them away).
  - ``Vehicle.teleport`` now supports changing rotation without resetting the vehicle
  - ``BeamNGpy.open`` now always tries to connect to already running simulator no matter the value of the launch argument
  - ``Vehicle.switch``, ``Vehicle.focus`` to switch the simulator's focus to the selected vehicle
  - ``BeamNGpy.vehicles.spawn`` now has a new argument ``connect`` to allow for not connecting the newly spawned vehicle to BeamNGpy
  - ``Vehicle.recover`` to repair a vehicle and teleport it to a drivable position
  - ``BeamNGpy.vehicles.replace`` to replace a vehicle with another one at the same position
  - ``beamngpy.quat.quat_multiply`` utility function to multiply two quaternions
  - optimized the ``Camera`` sensor decoding to be faster
  - updated the required Python packages to newer versions
  - ``Vehicle.set_license_plate`` to set a license plate text for a vehicle
  - ``Vehicle.sensors.poll`` now allows also polling only a specified list of sensor names
  - ``BeamNGpy.disconnect`` to disconnect from the simulator without closing it
  - changed ``Camera`` sensor default parameters to not include annotation and depth data (for faster polling)
  - added the optional ``steps_per_second`` parameter to ``BeamNGpy.settings.set_deterministic``
  - ``BeamNGpy.control.return_to_main_menu`` to exit the currently loaded scenario
  - added the parameter ``quit_on_close`` to the BeamNGpy constructor. If set to ``False``, ``BeamNGpy.close`` will keep the simulator running.

- Bugfixes

    - ``Vehicle.state['rotation']`` now returns vehicle rotation consistent with the rest of the simulator. Previously, this rotation was rotated 180° around the Y axis.

      - ⚠️ if you are using ``Vehicle.state['rotation']`` in your existing scripts, you may need to flip it back for your intended use. You can use ``beamngpy.quat.quat_multiply((0, 0, 1, 0), <your_rotation>)`` for that purpose.
    - fixed the issue with BeamNGpy scenarios sometimes resetting and not working properly after loading
    - fixed ``Camera.extract_bounding_boxes`` not to crash on non-Windows systems
    - fixed ``beamng.scenario.start()`` not working when the simulator was paused with ``beamng.control.pause()`` before
    - fixed vehicle color and license plate text not being applied to dynamically spawned vehicles

- BeamNGpy protocol: added support for out-of-order protocol messages
- Deprecations

    - the ``remote`` argument of the ``BeamNGpy`` class is not used anymore

Version 1.24
============
- Major changes to the protocol communicating between BeamNG.tech and BeamNGpy

  - Be aware that versions of BeamNG.tech older than 0.26 are not compatible with BeamNGpy 1.24
    and older versions of BeamNGpy will not work with BeamNG.tech 0.26.
- Major updates to BeamNGpy sensor suite and its API

  - The public API of the ``Camera``, ``Lidar`` and ``Ultrasonic`` sensors changed heavily, please see
    the ``examples`` folder to see their usage.
- Accelerometer sensor now available
- Add support for loading TrackBuilder tracks
- Add support for loading Flowgraph scenarios
- Fix: multiple vehicles now do not share color in instance annotations
- Add ``Vehicle.teleport`` helper function which allows to teleport a vehicle directly through its instance
- ``BeamNGpy.open`` now tries to (re)connect to already running local instance
- Removed deprecated BeamNGpy functionality

  - ``setup_logging`` (superseded by ``set_up_simple_logging`` and ``config_logging``)
  - ``rot`` argument used for setting rotation of objects and vehicles in Euler angles, use ``rot_quat`` which expects quaternions
    (you can use the helper function ``angle_to_quat`` to convert Euler angles to quaternions)
  - ``update_vehicle`` function is removed
  - the ``requests`` argument in ``Vehicle.poll_sensors`` is removed
  - ``poll_sensors`` now does not return a value
  - the ``deploy`` argument of ``BeamNGpy.open`` is removed

Version 1.23.1
==============
- Add Feature Overview notebook
- Add argument checking to the IMU sensor
- Add support for Mesh Roads
- Add option to log BeamNGpy protocol messages
- Fix duplicate logging when calling ``config_logging`` multiple times

Version 1.23
============
- Fix semantic annotations (supported maps are Italy and ECA)
- Add option to teleport vehicle without resetting its physics state
- Add option to set velocity of a vehicle by applying force to it
- Support for updated ultrasonic sensor
- New sensor API - LiDAR, ultrasonic sensor
- Fix camera sensor creating three shared memories even when not needed
- Add BeamNGpy feature overview example notebook
- Remove research mod deployment and ``setup-workspace`` phase of setup
- (Experimental) Support for Linux BeamNG.tech servers

Version 1.22
============
- Hide menu on a scenario start
- Do not detach the state sensor on disconnecting a vehicle, as this disallows the reuse of vehicle objects
- Fix camera sensor logging error
- Fix 'Using mods with BeamNGpy' demo notebook

Version 1.21.1
==============
- Fix example notebooks

Version 1.21
============
- Fix and restructure ``logging`` usage
- Add more verbose logging
- Fix message chunking in networking
- Update examples/tests to address GridMap being gone
- Improve handling of userpath discovery and mod deployment

Version 1.20
============
- Adjust userpath handling according to changes in BeamNG.drive from 0.22 onwards
- Overhaul documentation style and structure
- Add function to set up userpath for BeamNG.tech usage
- Add multicam test
- Fix issue when multiple functions are waiting in researchGE.lua
- Fix instance annotations always being rendered even when not desired


Version 1.19.1
==============
- Swap client/server model to allow multiple BeamNGpy instances to connect to one running simulator simultaneously
- Add ``Level`` class representing a level in the simulation
- Change ``Scenario`` class to point to ``Level`` it is in
- Add ``get_levels``, ``get_scenarios``, ``get_level_scenarios``, ``get_levels_and_scenarios`` methods to ``BeamNGpy`` class to query available content
- Add ``get_current_scenario`` method to ``BeamNGpy`` class to query running scenario
- Add ``get_current_vehicles`` method to ``BeamNGpy`` class to query active vehicles
- Add ``SceneObject`` class to the ``scenario`` module as a basis for the various types of objects in a scene in BeamNG.tech, currently including ``DecalRoad``
- Add ``get_scenetree`` and ``get_scene_object`` methods to ``BeamNGpy`` class to enable querying objects in the active scene
- Add ``add_debug_spheres``, ``add_debug_polyline``, ``add_debug_cylinder``, ``add_debug_triangle``, ``add_debug_rectangle``, ``add_debug_text``, ``add_debug_square_prism`` methods to ``BeamNGpy`` class to visualize 3D gizmos in the simulator
- Add Inertial Measurement Unit sensors
- Add Ultrasonic Distance Measurement sensor
- Add noise module to randomize sensor data for cameras and lidars
- Add instance annotation option to ``Camera`` sensor including methods to ``extract_bboxes``, ``export_bbox_xml``, and ``draw_bboxes`` for bounding-box-related operations based on semantic and instance annotations (limited to vehicles right now)
- Add options to use only socket-based communication for ``Camera`` and ``Lidar`` sensor
- Add methods to configure BeamNG.tech's Vehicle Stats Logger from BeamNGpy
- Add FAQ to README
- Add Contributor License Agreement and guidelines
- Fix stray dependency on PyScaffold
- Fix lidar points being visible in camera sensor images

Version 1.18
============
- Add function to switch current viewport to the relative camera mode with options to control the position of the camera
- Add function to display debug lines in the environment
- Add function to send Lua commands to be executed inside the simulation

Version 1.17.1
==============
- Fix deterministic mode ignoring user-defined steps per second

Version 1.17
============
- Add ``change_setting`` and ``apply_graphics_setting`` methods including a usage example
- Add option to specify rotations as quaternions where appropriate
- Add example for querying the road network

Version 1.16.5
==============
- Fix prefab compilation

Version 1.16.4
==============
- Add ``teleport_scenario_object`` method to ``BeamNGpy`` class
- Update vehicle state example
- Fix decal road positioning
- Fix ``spawn_vehicle`` not setting color and license plate correctly
- Fix ``spawn_vehicle`` rotation in degrees

Version 1.16.3
==============
- Fix lidar visualizer using wrong buffer types in newer PyOpenGL version

Version 1.16.2
==============
- Update values of `Electrics` sensor not following our naming conventions
- Fix camera orientation issue
- Add example for using the `Camera` sensor like a multishot camera

Version 1.16.1
==============
- Fix spaces in vehicle names breaking the scenario prefab

Version 1.16
============
- Make BeamNGpy ship required Lua files and deploy them as a mod on launch
- Add traffic controls
- Add option to specify additional Lua extensions to load per vehicle
- Add ``set_lights`` method to vehicle class
- Add test for setting lights
- Add test for vehicle bounding box
- Add ``over_objects`` field to ``Road`` class
- Fix lack of `__version__`
- Fix electrics sensor not returning values directly
- Fix `ai_set_script` teleporting vehicle

Version 1.15
============
- Add option to pass additional Lua extensions to be loaded on startup
- Fix waiting for vehicle spawn after changing parts to hang infinitely

Version 1.13
============
- Add option to disable node interpolation on roads
- Add `get_bbox()` method to `Vehicle` class

Version 1.12
============
- Add option to specify road ID for placed DecalRoads

Version 1.11
============
- Add ``StaticObject`` class to scenario module that allows placement of
  static meshes
- Add option for visualization to the Lidar sensor
- Add helper functions to query scenario for certain objects in the world
- Add example notebook showcasing procedural mesh and static mesh placement
  including a scenario camera
- Fix vehicle state not being synchronized properly
- Fix scenario unloading glitch
- Fix ``ai_drive_in_lane`` not updating GUI state correctly
- Fix camera sensor showing residual head-/taillight flare

Version 1.10
============
- Add functions to spawn/despawn vehicles during a scenario
- Add script AI function to vehicle and update AI line example accordingly
- Add function to change AI aggression
- Add functions to place procedurally generated primitives in the environment
- Add unit tests for sensors, scenarios, and vehicles
- Fix scenario not being cleared when BeamNG instance is closed

Version 1.9.1
=============
- Make scenario generation & loading respect user path setting

Version 1.9
===========
- Add function to switch active vehicle
- Add function to set position & orientation of the ingame camera

Version 1.8
===========
- Add vehicle teleporting function to ``BeamNGpy`` class
- Add time of day control
- Add function to switch weather presets
- Add function to await vehicle spawns
- Expose part configuration options of vehicles
- Expose current part configuration of vehicles
- Add function to change part configuration of vehicles
- Add function to change vehicle colour
- Add more documentation

Version 1.7.1
=============
- Make ai methods switch to appropriate modes

Version 1.7
===========
- Add manual gear control
- Add shift mode control

Version 1.6
===========
- Add option to set target waypoint for builtin vehicle AI
- Make shmem handle unique OS-wide

Version 1.5
===========
- Add ``get_gamestate()`` to ``BeamNGpy`` class
- Make vehicle state being synched upon initial connection
- Fix vehicle state not being updated on poll if only gameengine-specific
  sensors were attached.

Version 1.4
===========
- Add vehicle-level state updates
- Rework code to work with existing scenarios/vehicles

Version 1.3
===========
- Add support to specify polyline with per-vertex speed to the AI

Version 1.2
===========
- Add wait option to step function in ``beamng.py``

Version 1.1
===========
- Add basic Lidar point cloud visualiser
- Add AI control to vehicles
- Add option to attach cameras to scenarios to render frames relative to
  world space

Version 1.0
===========

- Restructure code to offer modular sensor model
- Implement scenario class to specify and generate BeamNG scenarios
- Implement vehicle class that offers control over vehicles and ways  to
  dynamically de-/attach sensors
- Implement shared memory communication to boost performance
- Add Camera sensor with colour, depth, and annotation data
- Add multi-cam support
- Add lidar sensor
- Add G-Force sensor
- Add damage sensor
- Add electrics sensor
- Add control over simulation timescale and stepping through simulation at
  fixed rates
- Add example code demonstrating scenario specification with control of a
  vehicle that has various sensors attached

Version 0.4
===========
- Add ``move_vehicle()`` method.

Version 0.3.6
=============
- Pass configured host and port to BeamNG.drive process.

Version 0.3.5
=============
- Fix ``close()`` in ``BeamNGPy`` not checking if there's even a process to be
  killed.

Version 0.3.4
=============
- Fix messages being split incorrectly when the message happened to contain a
  newline through msgpack encoding.

Version 0.3.3
=============
- Make ``BeamNGPy`` class take ``**options`` and add ``console`` as one to allow
  running BeamNG.drive with the console flag.

Version 0.3.2
=============
- Make BeamNGpy assume a running instance if binary is set to ``None``
- Add option to change vehicle cursor

Version 0.3.1
=============
- Add ``restart_scenario`` method to restart a running scenario

Version 0.3
===========
- Add method to pause simulation
- Add method to resume simulation

Version 0.2
===========
- Add option to specify image size when requesting vehicle state
- Add blocking method to get vehicle state
- Add method to set relative camera
- Add methods to hide/show HUD
- Default to realistic gearbox behaviour
- Add ``gear`` property to vehicle state
- Add ``gear`` as an option to vehicle input representing the gear the vehicle
  is supposed to shift to.

Version 0.1.2
=============
- Remove fstrings from documentation
- Add option to override BeamNG.drive binary being called

Version 0.1
===========
- Basic IPC and example functions

