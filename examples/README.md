# Examples

To help you getting started wih our library, we have added a collection of examples illustrating our features.
This guide helps exploring the collection and can help in finding examples for specific problems and features.

For getting started we suggest taking a look at the following examples:
* [Feature Overview][21]
* [Scenario Control][25]
* [East Coast Random][20]
* [Vehicle Road Bounding Box][17]
* [Annotation and Bounding Boxes][5]

More quick usage examples can be found in our [test suite][1].

## AI, traffic, and path execution

Quick map of the public APIs for driving paths, Script AI, traffic, and checkpoints. Prefer the linked example when you want a full runnable script.

Ordered least → most changes to the vehicle/scene: waypoints → manual points → paths → Script AI Editor → traffic.

### Waypoints

Named map waypoints, light AI mode tweaks, and scenario checkpoints.

| API | Example | Use case | Example file |
| ------------- |-------------|-------------|-------------|
| ``Vehicle.ai.set_waypoint`` / ``set_speed`` / ``set_aggression`` | ``vehicle.ai.set_waypoint("wp1"); vehicle.ai.set_speed(20, mode="limit")`` | italy (Feature Overview) — drive to one waypoint with a speed/aggression cap | [Feature Overview][21] |
| ``Vehicle.ai.set_mode`` | ``vehicle.ai.set_mode("span")`` or ``"traffic"`` / ``"disabled"`` | italy — built-in AI modes (explore map, act as traffic, turn AI off) | [Feature Overview][21] |
| ``Scenario.add_checkpoints`` | ``scenario.add_checkpoints(positions, scales, ids=[...], draw_debug=True)`` | tech_ground — place checkpoint waypoints for a custom scenario (optional debug spheres) | [Checkpoints][7] |

### Manual points

Custom world-space polylines you define (no navgraph).

| API | Example | Use case | Example file |
| ------------- |-------------|-------------|-------------|
| ``Vehicle.ai.set_line`` | ``vehicle.ai.set_line([{"pos": (x, y, z), "speed": 12.0}, ...])`` | tech_ground — follow a world-space polyline with per-point speed (no navgraph) | [AI Set Line][50] |

### Paths

Navgraph routes, waypoint lists, and timed node scripts.

| API | Example | Use case | Example file |
| ------------- |-------------|-------------|-------------|
| ``Vehicle.ai.drive_using_waypoints`` | ``vehicle.ai.drive_using_waypoints(["wp1", "wp2"], wp_speeds={"wp1": 25.0}, route_speed_mode="limit", aggression=0.5)`` | hirochi_raceway — lap named navgraph waypoints (speeds, aggression) | [AI Waypoints][27] |
| ``Vehicle.ai.set_script`` | ``vehicle.ai.set_script([{"x": ..., "y": ..., "z": ..., "t": ...}, ...])`` | west_coast_usa — follow a timed sine-path polyline (speed from timestamps) | [AI Set Script][4] |
| ``Vehicle.ai.get_initial_spawn_position_orientation`` | ``vehicle.ai.get_initial_spawn_position_orientation(script)`` | west_coast_usa — spawn pose for the start of that script path | [AI Set Script][4] |

### Script AI Editor

Record, import, and replay Script AI paths.

| API | Example | Use case | Example file |
| ------------- |-------------|-------------|-------------|
| ``Vehicle.ai.start_recording`` / ``stop_recording`` | ``vehicle.ai.start_recording(record_speed=False); ...; vehicle.ai.stop_recording("path.json")`` | smallgrid — record a Script AI path (``t`` timestamps, or ``v`` speeds if ``record_speed=True``) | [Script AI Editor][53] |
| ``Vehicle.ai.import_script_ai_file`` | ``script = vehicle.ai.import_script_ai_file("path.json")`` | smallgrid — load a Script AI Editor / recorded JSON path | [Script AI Editor][53] |
| ``Vehicle.ai.execute_script`` | ``vehicle.ai.execute_script(script, cling=True, start_delay=1.0)`` | smallgrid — replay a Script AI path (incl. shipped t-/v-based samples) | [Script AI Editor][53] |

### Traffic

Spawn, configure, and control traffic vehicles.

| API | Example | Use case | Example file |
| ------------- |-------------|-------------|-------------|
| ``BeamNGpy.traffic.spawn`` | ``bng.traffic.spawn(max_amount=10)`` | italy — spawn AI traffic vehicles on the map | [Feature Overview][21] |
| ``BeamNGpy.traffic.start`` / ``stop`` | ``bng.traffic.start([veh_a, veh_b]); bng.traffic.stop(stop=True)`` | tech_ground + ``add_simple_road_network`` — hand already-spawned vehicles on roads to traffic AI | [Traffic Participants][52] |
| ``BeamNGpy.traffic.reset`` | ``bng.traffic.reset()`` | Teleport traffic away from the player (clear jams) | (API only) |
| ``TrafficConfig`` | ``TrafficConfig(bng, "/tech/traffic_config_ex.json")`` | Load a traffic + vehicle setup from a JSON config | [Traffic Configuration][43] |

Related helpers on the same AI object: ``drive_in_lane``, ``set_avoid_cars``, ``set_target`` (chase/flee). See ``beamngpy.api.vehicle.ai.AIApi`` for full signatures.

| Name     | How to           |
| ------------- |-------------|
|[Feature Overview][21]|<ul><li>see the basics of what can be done with BeamNGpy</li></ul>|
|[Scenario Control][25]|<ul><li>understand the different ways of creating, loading and controlling scenarios</li></ul>|
|[Modding Interface][2]|<ul><li>write and deploy a mod that interacts with BeamNGpy</li><li>write python functions that interact with functions of the custom mod</li></ul>|
|[Road Network][3]|<ul><li>fetch information about the current level's road network</li></ul>|
|[AI Set Script][4]|<ul><li>west_coast_usa — timed ``set_script`` sine path; spawn pose helper</li></ul>|
|[AI Set Line][50]|<ul><li>tech_ground — ``set_line`` world polyline + speed (no navgraph)</li></ul>|
|[Script AI Editor][53]|<ul><li>smallgrid — Script AI Editor record / import / replay</li></ul>|
|[AI Waypoints][27]|<ul><li>hirochi_raceway — ``drive_using_waypoints`` laps with speeds and aggression</li></ul>|
|[Environment][51]|<ul><li>west_coast_usa — time of day, gravity, weather presets</li></ul>|
|[Traffic Participants][52]|<ul><li>tech_ground + simple road network — ``traffic.start`` / ``stop`` on existing vehicles</li></ul>|
|[Annotation and Bounding Boxes][5]|<ul><li>get semantic annotations</li><li>get instance annotations</li><li>draw bounding boxes (note that this feature is not ready for use yet)</li></ul>|
|[Settings][6]|<ul><li>change the simulation's graphics settings</li></ul>|
|[Checkpoints][7]|<ul><li>tech_ground — add checkpoints to a custom scenario (``ids``, ``draw_debug``)</li></ul>|
|[West Coast LiDAR][8]|<ul><li>use the LiDAR sensor</li></ul>|
|[Multiple Clients][9]|<ul><li>connect multiple BeamNGpy instances to the simulation</li></ul>|
|[Multishot Camera][10]|<ul><li>change the position and rotation of a camera</li></ul>|
|[Object Placement][11]|<ul><li>define a custom scenario for a given map</li><li>generate procedural content, i.e. simple meshes</li></ul>|
|[Procedural Meshes][12]|<ul><li>generate procedural content</li></ul>|
|[Road Definition][13]|<ul><li>add custom roads to given maps</li></ul>|
|[Advanced Driver Comfort Analysis][22]|<ul><li> use the Advanced IMU sensor</li></ul>|
|[Spawning][15]|<ul><li>spawn a new vehicle after starting the scenario</li></ul>|
|[Ultrasonic Sensor][16]|<ul><li>use the ultrasonic sensor</li></ul>|
|[Vehicle Road Bounding Box][17]|<ul><li>create a custom road</li><li>extract road information from the simulation</li><li>get a vehicle's bounding box</li></ul>|
|[Vehicle State Plotting][18]|<ul><li>use the state sensor</li></ul>|
|[East Coast Random][20]|<ul><li>create a simple scenario</li><li>use the simulator's AI with BeamNGpy</li></ul>|
|[Powertrain Analysis][23]|<ul><li>use the Powertrain sensor</li></ul>|
|[Road Network Exporter][24]|<ul><li>Export BeamNG maps as .xodr (OpenDRIVE), .osm (OpenStreetMap), or SUMO (.nod.xml / .edg.xml).</li><li>The exported road networks contain elevation and road wideness data, along with junction connectivity.</li><li>After a SUMO export, convert with ``netconvert`` (optional native ``.netccfg`` / junction flags), generate trips with ``randomTrips.py``, and load in ``sumo-gui`` to check that the network carries traffic (not a live BeamNG–SUMO co-simulation).</li></ul>|
|[Road Network Importer][40]|<ul><li>Import a road network from different formats</li></ul>|
|[Platooning][26]|<ul><li>form a vehicle platooning formation with BeamNGpy</li></ul>|
|[Parking Assist and Blind Spot Detection][28]|<ul><li>get parking assistance</li><li>blind spot HUD notifications</li></ul>|
|[Lane-Keeping Assist][29]|<ul><li>automatically limit vehicle speed based on road curvature</li><li>force feedback on accidental lane exit</li></ul>|
|[West Coast Radar][14]|<ul><li>use the RADAR sensor</li></ul>|
|[West Coast IMU][19]|<ul><li>use the IMU sensor</li></ul>|
|[ACC Test][30]|<ul><li>use the radar to monitor adaptive cruise control behaviour</li></ul>|
|[Camera Streaming][31]|<ul><li>stream camera images</li></ul>|
|[Faster Than Realtime][32]|<ul><li>run the simulation at a faster rate than realtime</li></ul>|
|[GPS Trajectory][33]|<ul><li>get the GPS trajectory of a vehicle</li></ul>|
|[Heightmap Importer][34]|<ul><li>import a heightmap</li></ul>|
|[Ideal RADAR Sensor IDs Tracking][35]|<ul><li>track objects with the Ideal RADAR sensor</li></ul>|
|[Ideal RADAR Sensor Plot Data][36]|<ul><li>plot data from the Ideal RADAR sensor</li></ul>|
|[Import Peaks and Roads][37]|<ul><li>import peaks and roads</li></ul>|
|[Map Sensor Configuration][38]|<ul><li>polling all sensors of a configuration</li></ul>|
|[Radar Analysis][39]|<ul><li>analyze radar data</li></ul>|
|[Roads Plot][41]|<ul><li>plotting a road profile</li></ul>|
|[Small Grid IMU][42]|<ul><li>use the IMU sensor</li></ul>|
|[Traffic Configuration][43]|<ul><li>configure traffic</li></ul>|
|[Vehicle Logging][44]|<ul><li>log vehicle data</li></ul>|
|[Vehicle Mesh Data][45]|<ul><li>get vehicle mesh data</li></ul>|
|[Vehicle Sensor Configuration][46]|<ul><li>configure the vehicle sensors</li></ul>|
|[Headless Mode Camera Streaming][47]|<ul><li>stream camera images in headless mode (GPU required, no window)</li></ul>|
|[No-GPU Mode Stats][48]|<ul><li>step the sim in no-GPU mode and plot live position and speed</li></ul>|
|[Traffic Signals Read Write][49]|<ul><li>read and override traffic signal lamp states and controller durations</li></ul>|

[1]: https://github.com/BeamNG/BeamNGpy/tree/master/tests
[2]: https://github.com/BeamNG/BeamNGpy/tree/master/examples/modInterface
[3]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/access_road_network.ipynb
[4]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/ai_set_script.py
[5]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/annotation_bounding_boxes.ipynb
[6]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/change_settings.py
[7]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/checkpoints.py
[8]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/west_coast_lidar.py
[9]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/multi_client.ipynb
[10]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/multishot_camera.ipynb
[11]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/object_placement.ipynb
[12]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/procedural_meshes.py
[13]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/road_definition.py
[14]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/west_coast_radar.py
[15]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/spawning.ipynb
[16]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/ultrasonic_demo.py
[17]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/vehicle_road_bounding_box.ipynb
[18]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/vehicle_state_plotting.ipynb
[19]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/west_coast_IMU.py
[20]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/east_coast_random.py
[21]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/feature_overview.ipynb
[22]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/advanced_comfort_analysis.ipynb
[23]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/powertrain_data.ipynb
[24]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/road_network_exporter.py
[25]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/scenario_control.ipynb
[26]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/platooning.py
[27]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/ai_waypoints.py
[28]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/adas_ultrasonic.py
[29]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/lka_example.py
[30]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/acc_test.py
[31]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/camera_streaming.py
[32]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/faster_than_realtime.py
[33]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/GPS_trajectory.py
[34]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/heightmap_importer.py
[35]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/IdealRADARSensor_IDs_tracking.py
[36]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/IdealRADARSensor_plot_data.py
[37]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/import_peaks_and_roads.py
[38]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/map_sensor_configuration.py
[39]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/radar_analysis.ipynb
[40]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/road_network_importer.py
[41]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/roads_plot.ipynb
[42]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/smallgrid_IMU.py
[43]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/traffic_configuration.py
[44]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/vehicle_logging.py
[45]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/vehicle_mesh_data.py
[46]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/vehicle_sensor_configuration.py
[47]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/headless_mode_camera_streaming.ipynb
[48]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/nogpu_mode_stats.ipynb
[49]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/traffic_signals_read_write.ipynb
[50]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/ai_set_line.py
[51]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/environment.py
[52]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/traffic_participants.py
[53]: https://github.com/BeamNG/BeamNGpy/blob/master/examples/script_ai_record_import_replay.py
