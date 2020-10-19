=========
Changelog
=========

=======
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

