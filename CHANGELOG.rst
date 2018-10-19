=========
Changelog
=========

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
