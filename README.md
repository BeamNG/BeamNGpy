# BeamNGpy


[![Documentation](https://github.com/BeamNG/BeamNGpy/raw/master/media/documentation.png)][1]

##### Table of Contents
[About](#about)
[Prerequisites](#prereqs)
[Installation](#installation)
[Usage](#usage)
[Troubleshooting](#troubleshooting)

<a name="about" ></a>

## About

BeamNGpy is an official library providing a Python interface to BeamNG.tech,
the academia- and industry-oriented fork of the video game [BeamNG.drive][4].
BeamNGpy and BeamNG.tech are designed to go hand in hand, both being kept up
to date to support each other's functions, meaning using the latest versions
of both is recommended.

It allows remote control of the simulation, including vehicles contained in it:

![Throttle control](https://github.com/BeamNG/BeamNGpy/raw/master/media/throttle.gif)
![Steering control](https://github.com/BeamNG/BeamNGpy/raw/master/media/steering.gif)

Vehicles and the environment can be equipped with various sensors that provide
simulated sensor data such as a camera feed, with options for depth values and
pixel-perfect semantic annotation or a simulated Lidar sensor:

![Multiple cameras](https://github.com/BeamNG/BeamNGpy/raw/master/media/camera.png)
![Lidar](https://github.com/BeamNG/BeamNGpy/raw/master/media/lidar.gif)

<a name="prereqs"></a>

## Prerequisites

Usage of BeamNGpy requires BeamNG.tech to be installed. Builds of
BeamNG.tech are made available for non-commercial use upon request using
[this form][2]. For commercial use, contact us at [licensing@beamng.gmbh][3].
Once downloaded (and extracted, depending on whether or no BeamNG.tech was
obtained as a `.zip`), you can set an environment variable `BNG_HOME` to where
BeamNG.tech can be run from, or provide a path to the BeamNGpy library
during initialization.

The regular [Steam release of BeamNG.drive][4] is compatible to an extent as
well. Certain sensors like the simulated LiDAR or camera will not work, but
most of the functions that are not exclusive to a Tech build will likely
work.

<a name="installation"></a>

## Installation

The library itself is available on [PyPI][5] and can therefore be installed
using common methods like `pip`:

    pip install beamngpy

<a name="usage"></a>

## Usage

Once installed, the library can be imported using `import beamngpy`. A short
usage example setting up a scenario with one vehicle in the West Coast USA map
that spans the area is:

```python
from beamngpy import BeamNGpy, Scenario, Vehicle

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:64256
bng = BeamNGpy('localhost', 64256, home='/path/to/bng/tech')
# Launch BeamNG.tech
bng.open()
# Create a scenario in west_coast_usa called 'example'
scenario = Scenario('west_coast_usa', 'example')
# Create an ETK800 with the licence plate 'PYTHON'
vehicle = Vehicle('ego_vehicle', model='etk800', licence='PYTHON')
# Add it to our scenario at this position and rotation
scenario.add_vehicle(vehicle, pos=(-717, 101, 118), rot=None, rot_quat=(0, 0, 0.3826834, 0.9238795))
# Place files defining our scenario for the simulator to read
scenario.make(bng)

# Load and start our scenario
bng.load_scenario(scenario)
bng.start_scenario()
# Make the vehicle's AI span the map
vehicle.ai_set_mode('span')
input('Hit enter when done...')
```

More examples can be found in the [examples/][6] folder of this repository and
the documentation of the library is [available here.][7].

## Troubleshooting

This section lists common issues with BeamNGpy in particular. Since this
library is closely tied to BeamNG.tech and thus BeamNG.drive, it is also
recommended to consult the documentation on BeamNG.drive here:

[https://documentation.beamng.com/][8]

### BeamNGpy cannot establish a connection

 - Make sure BeamNG.tech and Python are allowed to connect to your current
   network in Windows Firewall.

### BeamNG.tech quietly fails to launch

- There is a known issue where BeamNG.tech quietly crashes when there is a
  space in the configured userpath. Until this issue is fixed, it is
  recommended to either switch to a path that does not contain a space or
  change the userpath directly in the "startup.ini" file located in the
  directory of your BeamNG.tech installation.

[1]: https://beamngpy.readthedocs.io/en/latest/
[2]: https://register.beamng.tech/
[3]: mailto:licensing@beamng.gmbh
[4]: https://store.steampowered.com/app/284160/BeamNGdrive/
[5]: https://pypi.org/project/beamngpy/
[6]: https://github.com/BeamNG/BeamNGpy/tree/master/examples
[7]: https://beamngpy.readthedocs.io/en/latest/
[8]: https://documentation.beamng.com/
