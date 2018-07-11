=========
Changelog
=========

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
