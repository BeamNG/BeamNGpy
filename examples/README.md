# Example Guide

To help you getting started wih our library, we have added a collection of examples illustrating our features.
This guide helps exploring the collection and can help in finding examples for specific problems and features.

For getting started we suggest taking a look at the following examples:
* [Feature Overview][21]
* [East Coast Random][20]
* [Vehicle Road Bounding Box][17]
* [Annotation and Bounding Boxes][5]

More quick usage examples can be found in our [test suite][1].

| Name     | How to           |
| ------------- |-------------|
|[Feature Overview][21]|<ul><li>see the basics of what can be done with BeamNGpy</li></ul>|
|[Modding Interface][2]|<ul><li>write and deploy a mod that interacts with BeamNGpy</li><li>write python functions that interact with functions of the custom mod</li></ul>|
|[Road Network][3]|<ul><li>fetch information about the current level's road network</li></ul>|
|[AI Line][4]|<ul><li>use the simulator's AI with BeamNGpy</li></ul>|
|[Annotation and Bounding Boxes][5]|<ul><li>get semantic annotations</li><li>get instance annotations</li><li>draw bounding boxes (note that this feature is not ready for use yet))</li></ul>|
|[Settings][6]|<ul><li>change the simulation's graphics settings</li></ul>|
|[Checkpoints][7]|<ul><li>add checkpoints to custom scenarios</li></ul>|
|[LiDAR][8]|<ul><li>use the LiDAR sensor</li></ul>|
|[Multiple Clients][9]|<ul><li>connect multiple BeamNGpy instances to the simulation</li></ul>|
|[Multishot Camera][10]|<ul><li>change the position and rotation of a camera</li></ul>|
|[Object Placement][11]|<ul><li>define a custom scenario for a given map</li><li>generate procedural content, i.e. simple meshes</li></ul>|
|[Procedural Meshes][12]|<ul><li>generate procedural content</li></ul>|
|[Road Definition][13]|<ul><li>add custom roads to given maps</li></ul>|
|[Simple Driver Comfort Analysis][14]|<ul><li>use the IMU sensor</li></ul>|
|[Advanced Driver Comfort Analysis][22]|<ul><li> use the Advanced IMU sensor</li></ul>|
|[Spawning][15]|<ul><li>spawn a new vehicle after starting the scenario</li></ul>|
|[Ultrasonic Sensor][16]|<ul><li>use the ultrasonic sensor</li></ul>|
|[Vehicle Road Bounding Box][17]|<ul><li>create a custom road</li><li>extract road information from the simulation</li><li>get a vehicle's bounding box</li></ul>|
|[Vehicle State Plotting][18]|<ul><li>use the state sensor</li></ul>|
|[West Coast LiDAR][19]|<ul><li>use the LiDAR sensor</li></ul>|
|[East Coast Random][20]|<ul><li>create a simple scenario</li><li>use the simulator's AI with BeamNGpy</li></ul>|
|[Powertrain Analysis][23]|<ul><li>use the Powertrain sensor</li></ul>|
|[Road Network Exporter][24]|<ul><li>Export BeamNG maps as .xodr files (OpenDRIVE).</li><li>The exported road networks contain elevation and road wideness data, along with junction connectivity.</li><li>BeamNGpy also includes a new class with which to analyse the road network data oneself, and process it as required.</li></ul>|



[1]: https://github.com/BeamNG/BeamNGpy/tree/master/tests
[2]: https://github.com/BeamNG/BeamNGpy/tree/master/examples/modInterface
[3]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/access_road_network.ipynb
[4]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/ai_line.py
[5]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/annotation_bounding_boxes.ipynb
[6]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/change_settings.py
[7]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/checkpoints.py
[8]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/lidar_tour.py
[9]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/multi_client.ipynb
[10]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/multishot_camera.ipynb
[11]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/object_placement.ipynb
[12]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/procedural_meshes.py
[13]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/road_definition.py
[14]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/simple_driver_comfort_analysis.ipynb
[15]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/spawning.ipynb
[16]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/ultrasonic_demo.py
[17]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/vehicle_road_bounding_box.ipynb
[18]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/vehicle_state_plotting.ipynb
[19]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/west_coast_lidar.py
[20]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/east_coast_random.py
[21]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/feature_overview.ipynb
[22]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/advanced_comfort_analysis.ipynb
[23]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/powertrain_data.ipynb
[24]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/road_network_exporter.py
