========
beamngpy
========

Python API for BeamNG.research.

Installation
============

- Make sure `BeamNG.research.x64.exe` is in your `PATH`
- Make sure `git` is in your `PATH`
- Optionally, activate a virtualenv to install BeamNGPy into
- Navigate to the project directory
- Run `pip install -e .` to install the package

Building the docs
=================
The documentation is built with Sphinx. If wanted, Sphinx can be installed by issuing `pip install sphinx` . The docs
can then be built by running `sphinx-build docs html` in the project directory. This will place `.html` files for the
API reference in the folder `html`. The main entry point here is `html/index.html`.

Usage
=====

After installaton, the package can be imported with:

.. code-block:: python

  import beamngpy

The main class that exposes the API is `BeamNGPy` which can be instantiated with:

.. code-block:: python

  bpy = beamnpy.BeamNGPy('localhost', 64256)

Where host and port are your choice, of course. More details on the methods of this class can be found in the API reference.