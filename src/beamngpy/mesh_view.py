import math

from beamngpy import vec3

class Mesh_View:

    def __init__(self, mesh, mass_min = 0.0, mass_max = 50.0, force_min = 0.0, force_max = 50.0, vel_min = 0.0, vel_max = 50.0, vel_dir_sensitivity = 8.0, stress_min = 0.0,
        stress_max = 50.0, top_center = vec3(450.0, 750.0), top_scale = vec3(250.0, 250.0), front_center = vec3(450.0, 250.0), front_scale = vec3(250.0, 250.0),
        right_center = vec3(1350.0, 250.0), right_scale = vec3(250.0, 250.0), is_top = True, is_front = True, is_right = True, data_mode = 'mass'):
        """
        Constructs a mesh data manager and visualizer instance, used to generate multi-perspective vehicle mesh plots (nodes and beams), highlighting the distribution
        of various vehicle properties across the mesh (mass, force, velocity, stress).
        The resulting visualization will consist of up to three simultaneous views of the vehicle mesh:
            top: A top-down 'plan' view of the vehicle mesh (the XY-plane).
            front: A frontal view of the vehicle mesh (the XZ-plane).
            right: A right view of the vehicle mesh (the YZ-plane).
        The visualization can be coloured using various vehicle properties:
            mass: The mass distribution across the vehicle. This is fixed for all time. The colours are averaged for each beam.
            force: The force distribution across the vehicle. This is averaged for each beam, and we show the force vector magnitude.
            velocity: The velocity distribution across the vehicle.  This is averaged for each beam, and we show the velocity vector magnitude.
            vel_dir: The velocity direction distrubution.  This is the dot product between the vehicle forward direction and the mean velocity on the beam.
            beam stress: The stress distribution across the vehicle. This is a scalar value for each beam.
        On construction, the basic display properties are set, then when new data is generated, this can be updated using the 'update()' method.
        When we want to render the mesh plots, we call the 'display()' method to fetch the latest geometric data (rectangles and colored lines, here).
        The vehicle mesh visualizer can be paused using the 'pause()' method, which will freeze data on the screen until pause is unset using the same method.
        It is up to the user of this class how this information is rendered and with which graphics API.

        Args:
            mesh (Mesh): The Mesh sensor class instance.
            mass_min (float): The minimum displayable value for mass distribution plots.
            mass_max (float): The maximum displayable value for mass distribution plots.
            force_min (float): The minimum displayable value for force distribution plots.
            force_max (float): The maximum displayable value for force distribution plots.
            vel_min (float): The minimum displayable value for velocity distribution plots.
            vel_max (float): The maximum displayable value for velocity distribution plots.
            vel_dir_sensitivity (float): The division factor by which to limit the maximum angle when in velocity direction mode (ie max_ang := pi / vel_dir_sensitivity).
            stress_min (float): The minimum displayable value for beam stress distribution plots.
            stress_max (float): The maximum displayable value for beam stress distribution plots.
            top_center (vec3): The center point at which to display the top-view mesh, if selected. This is where the mesh origin node will appear on-screen.
            top_scale (vec3): The scaling factor (x, y, -) by which to scale the top-view mesh, if selected.
            front_center (vec3): The center point at which to display the front-view mesh, if selected. This is where the mesh origin node will appear on-screen.
            front_scale (vec3): The scaling factor (x, y, -) by which to scale the front-view mesh, if selected.
            right_center (vec3): The center point at which to display the right-view mesh, if selected. This is where the mesh origin node will appear on-screen.
            right_scale (vec3): The scaling factor (x, y, -) by which to scale the right-view mesh, if selected.
            is_top (bool): A flag which indicates if the top (plan) view will be computed.
            is_right (bool): A flag which indicates if the right (side) view will be computed.
            is_front (bool): A flag which indicates if the front view will be computed.
            data_mode (str): A string which indicates which data to show in the colour dimension. Valid values are { 'mass', 'force', 'velocity', 'vel_dir', 'stress' }.
        """

        # The mesh data structures.
        self.mesh = mesh                                                                    # The Mesh sensor class instance.
        self.mesh_data = {}                                                                 # The latest physical property readings at each node and beam.
        self.top_coords = { 'nodes' : {}, 'beams' : [] }                                    # The 2D projections of the nodes/beams in each view (top, front, right).
        self.front_coords = { 'nodes' : {}, 'beams' : [] }
        self.right_coords = { 'nodes' : {}, 'beams' : [] }
        self.beam_colors = []                                                               # The color list for the beams (depends on which data is being visualized).
        self.map = []                                                                       # A map from beams to node indices, eg { beam i = [node a, node b], ... }

        # Data ranges.
        self.mass_min, self.mass_max = mass_min, mass_max                                   # Masses will be clamped to range [mass_min, mass_max].
        self.force_min, self.force_max = force_min, force_max                               # Forces will be clamped to range [force_min, force_max].
        self.vel_min, self.vel_max = vel_min, vel_max                                       # Velocities will be clamped to range [vel_min, vel_max].
        self.stress_min, self.stress_max = stress_min, stress_max                           # Beam stresses will be clamped to range [stress_min, stress_max].
        self.mass_range_inv = 1.0 / (self.mass_max - self.mass_min)                         # The reciprocals of the data ranges (stored for efficiency).
        self.force_range_inv = 1.0 / (self.force_max - self.force_min)
        self.vel_range_inv = 1.0 / (self.vel_max - self.vel_min)
        self.stress_range_inv = 1.0 / (self.stress_max - self.stress_min)

        # Visualization parameters.
        self.top_center, self.top_scale = top_center, top_scale                             # Translation and Scale for each of the three views (top, front, right).
        self.front_center, self.front_scale = front_center, front_scale
        self.right_center, self.right_scale = right_center, right_scale
        self.data_mode = data_mode                                                          # Determines which data to visualise (mass, force, velocity, stress).
        self.is_top, self.is_front, self.is_right = is_top, is_front, is_right              # View flags.

        # The vehicle position and orthonormal frame of reference.
        self.forward = vec3(0.0, -1.0, 0.0)
        self.up = vec3(0.0, 0.0, -1.0)
        self.left = vec3(1.0, 0.0, 0.0)
        self.right = vec3(-1.0, 0.0, 0.0)

        # Miscellaneous.
        self.is_pause = False
        self.ang_range = 0.0                                                                # The angle range when using velocity direction mode ('vel_dir').
        self.force_range = 0.0                                                              # The force range when using force mode ('force').
        self.max_vd_ang = math.pi / float(vel_dir_sensitivity)                              # The maximum angle to detect when in velocity direction mode.
        self.max_vd_ang_inv = 1.0 / self.max_vd_ang

    def _project(self, nodes, beams, unit_n, unit_x, center, scale):
        """
        Projects the vehicle mesh to a 2D plane.

        Args:
            nodes: The collection of vehicle mesh nodes.
            beams: The collection of vehicle mesh beams.
            unit_n: The unit normal vector of the plane to project to (this must be normalized).
            unit_x: The unit x-axis vector on the plane to project to (this must be normalized).
            center: The center of the plane (used to translate the mesh on-screen into the correct position).
            scale: The factor by which to scale the projected mesh (used to fill out a required amount of space on-screen).

        Returns:
            Dict: The projected node coordinates and projected beam line coordinates.
        """
        projected_points = {}
        for k, v in nodes.items():
            node = v['pos']
            p = vec3(node['x'], node['y'], node['z'])
            x = p.dot(unit_x)
            y = p.dot(unit_n.cross(unit_x))
            projected_points[k] = [(x * scale.x) + center.x, (y * scale.y) + center.y]

        lines = []
        for _, v in beams.items():
            p1 = projected_points[v[0]]
            p2 = projected_points[v[1]]
            lines.append([p1[0], p1[1], p2[0], p2[1]])

        return { 'nodes' : projected_points, 'beams' : lines }

    def _compute_map(self, beams):
        """
        Computes a map from the collection of beams to the collection of nodes.
        eg for each beam, we have two node indices which are valid in the collection of nodes.
        This is used for fast access to coordinate data when performing beam computations.

        Args:
            beams: The collection of vehicle mesh beams.

        Returns:
            List: The map from beams to nodes.
        """
        map = []
        for _, v in beams.items():
            map.append([v[0], v[1]])
        return map

    def _get_color(self, val, val_min, val_max, range_inv):
        """
        Computes an RGB color in the blue -> green -> red colorbar range.

        Args:
            val: The data value to compute the color for.
            val_min: The minimum displayable value for the data (will be blue on the resulting plot).
            val_max: The maximum displayable value for the data (will be red on the resulting plot).
            range_inv: The reciprocal of the range ie 1.0 / (val_max - val_min). Used only for efficient computation.

        Returns:
            List: A list containing the R, G, B components of the computed colour, each in the range [0, 1].
        """
        val_snapped = max(val_min, min(val_max, val))                                       # Snap the data value to [val_min, val_max].
        v = (val_snapped - val_min) * range_inv                                             # Re-scale the data to range [0, 1].
        r, g, b = 0.0, 0.0, 0.0
        if v < 0.5:                                                                         # Val is in lower half, so interpolate between blue and green.
            b = (0.5 - v) * 2.0
            g = v * 2.0
        else:                                                                               # Val is in upper half, so interpolate between green and red.
            g = (1.0 - v) * 2.0
            r = (v - 0.5) * 2.0
        return [r, g, b]

    def pause(self, is_pause):
        """
        Sets whether to pause this time series instance or not. If paused, no updates will be performed and the data will remain frozen on screen.

        Args:
            is_pause (bool): Pauses the time series instance if True, otherwise sets it to continue updating.
        """
        self.is_pause = is_pause

    def change_mode(self, mode):
        """
        Sets the mode of data to be displayed on the vehicle mesh, using color. This should be one of: { 'mass', 'force', 'velocity', 'stress' }.

        Args:
            mode (str): The mode to set this vehicle mesh visualizer to.
        """
        self.top_coords, self.front_coords, self.right_coords = {'nodes' : {}, 'beams' : []}, {'nodes' : {}, 'beams' : []}, {'nodes' : {}, 'beams' : []}
        self.data_mode = mode

    def update(self, dir=vec3(0.0, -1.0, 0.0)):
        """
        Updates the data in this mesh visualizer to reflect the latest data from the associated Mesh sensor.
        NOTE: If the mesh visualizer is paused, the data will remain as it was previously.

        Args:
            dir (vec3): The forward direction of the vehicle (optional).
        """

        # If we are pausing, do not perform any data updating.
        if self.is_pause == True:
            return

        # Update the data structures to have the latest readings from the mesh.
        if self.mesh.vehicle.state:

            # Update the mesh sensor and get the latest nodes and beams data.
            self.mesh_data = self.mesh.poll()
            nodes = self.mesh.node_positions
            beams = self.mesh.beams

            # Compute the beams-to-nodes map (contains the two node indices for each beam).
            self.map = self._compute_map(beams)

            # Project the 3D node positions to 2D coordinates for each active view (top, front, right) and get the node/beam coordinates.
            if self.is_top == True:
                self.top_coords = self._project(nodes, beams, self.up, self.forward, self.top_center, self.top_scale)
            if self.is_front == True:
                self.front_coords = self._project(nodes, beams, self.right, self.forward, self.front_center, self.front_scale)
            if self.is_right == True:
                self.right_coords = self._project(nodes, beams, self.forward, self.left, self.right_center, self.right_scale)

            # Compute the color of each beam, depending on the currently-chosen data mode.
            self.beam_colors = []
            num_lines = len(beams)
            if self.data_mode == 'mass':
                node_data = self.mesh_data['nodes']
                for i in range(num_lines):
                    beam = self.map[i]
                    d1, d2 = node_data[beam[0]]['mass'], node_data[beam[1]]['mass']
                    avg = (d1 + d2) * 0.5
                    self.beam_colors.append(self._get_color(avg, self.mass_min, self.mass_max, self.mass_range_inv))
            elif self.data_mode == 'force':
                node_data = self.mesh_data['nodes']
                min_f, max_f = 1e12, -1e12
                for i in range(num_lines):
                    beam = self.map[i]
                    d1, d2 = node_data[beam[0]]['force'], node_data[beam[1]]['force']
                    mag_1, mag_2 = vec3(d1['x'], d1['y'], d1['z']).length(), vec3(d2['x'], d2['y'], d2['z']).length()
                    avg = (mag_1 + mag_2) * 0.5
                    min_f = min(min_f, avg)
                    max_f = max(max_f, avg)
                    self.beam_colors.append(self._get_color(avg, self.force_min, self.force_max, self.force_range_inv))
                self.force_range = max_f - min_f
            elif self.data_mode == 'velocity':
                node_data = self.mesh_data['nodes']
                for i in range(num_lines):
                    beam = self.map[i]
                    d1, d2 = node_data[beam[0]]['vel'], node_data[beam[1]]['vel']
                    mag_1, mag_2 = vec3(d1['x'], d1['y'], d1['z']).length(), vec3(d2['x'], d2['y'], d2['z']).length()
                    avg = (mag_1 + mag_2) * 0.5
                    self.beam_colors.append(self._get_color(avg, self.vel_min, self.vel_max, self.vel_range_inv))
            elif self.data_mode == 'vel_dir':
                node_data = self.mesh_data['nodes']
                min_ang, max_ang = 0.0, self.max_vd_ang
                for i in range(num_lines):
                    beam = self.map[i]
                    d1, d2 = node_data[beam[0]]['vel'], node_data[beam[1]]['vel']
                    mean_vec = vec3((d1['x'] + d2['x']) * 0.5, (d1['y'] + d2['y']) * 0.5, (d1['z'] + d2['z']) * 0.5)
                    dot = 1.0                           # default to forwards for very small velocities (to avoid jittering due to jumping velocity directions when stopped).
                    if mean_vec.length() > 0.2:
                        mean_n = mean_vec.normalize()
                        dot = mean_n.dot(vec3(dir[0], dir[1], dir[2]))
                    ang = abs(math.acos(max(-1, min(1, dot))))
                    ang = min(ang, self.max_vd_ang)
                    min_ang = min(min_ang, ang)         # Store the min/max angles in the mesh.
                    max_ang = max(max_ang, ang)
                    self.beam_colors.append(self._get_color(ang, 0.0, self.max_vd_ang, self.max_vd_ang_inv))
                self.ang_range = max_ang - min_ang
            else:
                beam_data = self.mesh_data['beams']
                for i in range(num_lines):
                    self.beam_colors.append(self._get_color(beam_data[i]['stress'], self.stress_min, self.stress_max, self.stress_range_inv))

    def display(self):
        """
        Gets all the display data for this vehicle mesh visualizer.

        Returns:
            dict: The coordinates for the nodes and beams for each of the three 2D projections (top, front, right), and the list of computed beam colors.
        """
        return { 'top' : self.top_coords, 'front' : self.front_coords, 'right' : self.right_coords, 'colors' : self.beam_colors }