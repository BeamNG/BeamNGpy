# BeamNGpy

[![BeamNG.research](https://github.com/BeamNG/BeamNGpy/raw/master/media/beamng.research.png)](https://beamng.gmbh/research/)
[![Documentation](https://github.com/BeamNG/BeamNGpy/raw/master/media/documentation.png)](https://beamngpy.readthedocs.io/en/latest/)

## About

BeamNGpy is an official library providing a Python interface to BeamNG.research, the research-oriented fork of the video game BeamNG.drive.

It allows remote control of the simulation, including vehicles contained in it:

![Throttle control](https://github.com/BeamNG/BeamNGpy/raw/master/media/throttle.gif)
![Steering control](https://github.com/BeamNG/BeamNGpy/raw/master/media/steering.gif)

Vehicles and the environment can be equipped with various sensors that provide simulated sensor data such as a camera feed, with options for depth values and pixel-perfect semantic annotation or a simulated Lidar sensor:

![Multiple cameras](https://github.com/BeamNG/BeamNGpy/raw/master/media/camera.png)
![Lidar](https://github.com/BeamNG/BeamNGpy/raw/master/media/lidar.gif)

## Prerequisites

Usage of BeamNGpy requires BeamNG.research to be installed. A copy free for non-commercial use can be obtained on the [BeamNG.research home page](https://beamng.gmbh/research/). Once downloaded (and extracted, depending on whether or no BeamNG.research was obtained as a `.zip`), you can set an environment variable `BNG_HOME` to where BeamNG.research can be run from, or
provide a path to the BeamNGpy library during initialisation.

## Installation

The library itself is available on [PyPI](https://pypi.org/project/beamngpy/) and can therefore be installed using common methods like `pip`:

    pip install beamngpy

## Usage

Once installed, the library can be imported using `import beamngpy`. A short usage example setting up a scenario with one vehicle in the West Coast USA map that spans the area is:

```python
from beamngpy import BeamNGpy, Scenario, Vehicle

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:64256
bng = BeamNGpy('localhost', 64256, home='/path/to/bng/research')
# Create a scenario in west_coast_usa called 'example'
scenario = Scenario('west_coast_usa', 'example')
# Create an ETK800 with the licence plate 'PYTHON'
vehicle = Vehicle('ego_vehicle', model='etk800', licence='PYTHON')
# Add it to our scenario at this position and rotation
scenario.add_vehicle(vehicle, pos=(-717, 101, 118), rot=(0, 0, 45))
# Place files defining our scenario for the simulator to read
scenario.make(bng)

# Launch BeamNG.research
bng.open()
# Load and start our scenario
bng.load_scenario(scenario)
bng.start_scenario()
# Make the vehicle's AI span the map
vehicle.ai_set_mode('span')
```

More examples can be found in the [examples/](https://github.com/BeamNG/BeamNGpy/tree/master/examples) folder of this repository and the documentation of the library is [available here.](https://beamngpy.readthedocs.io/en/latest/)
