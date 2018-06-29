========
BeamNGpy
========

Python API for BeamNG.research.

Installation
============

This API requires a copy of BeamNG.research, which is available for download
here:


https://beamng.gmbh/research/


Once obtained, make sure to add the BeamNG.research binaries to your ``PATH``
such that ``BeamNG.research.x64`` is callable from command line.

With that done, issue the following to install this library

.. code-block::

  pip install beamngpy

It's recommended to do this inside a VirtualEnv_.

Usage
=====

After installaton, the package can be imported with:

.. code-block:: python

  import beamngpy

The main class that exposes the API is ``BeamNGPy`` which can be instantiated
with:

.. code-block:: python

  bpy = beamngpy.BeamNGPy('localhost', 64256)

Where host and port are your choice, of course. More details on the methods of
this class can be found in the `API reference`_.

.. _VirtualEnv: https://docs.python.org/3/library/venv.html
.. _API reference: https://beamngpy.readthedocs.io/en/latest/
