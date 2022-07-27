from .scenario import ScenarioObject


class ProceduralMesh(ScenarioObject):
    def __init__(self, pos, rot, name, material, rot_quat=None):
        super(ProceduralMesh, self).__init__(name, name, 'ProceduralMesh',
                                             pos, rot, (1, 1, 1),
                                             rot_quat=rot_quat)
        self.material = material

    def place(self, bng):
        raise NotImplementedError()


class ProceduralCylinder(ProceduralMesh):
    def __init__(self, pos, rot, radius, height, name,
                 rot_quat=None, material=None):
        super(ProceduralCylinder, self).__init__(pos, rot, name, material,
                                                 rot_quat=rot_quat)
        self.radius = radius
        self.height = height

    def place(self, bng):
        bng.create_cylinder(self.name, self.radius, self.height, self.pos,
                            None, material=self.material, rot_quat=self.rot)


class ProceduralBump(ProceduralMesh):
    def __init__(self, pos, rot, width, length, height, upper_length,
                 upper_width, name, rot_quat=None, material=None):
        super(ProceduralBump, self).__init__(pos, rot, name, material,
                                             rot_quat=rot_quat)
        self.width = width
        self.length = length
        self.height = height
        self.upper_length = upper_length
        self.upper_width = upper_width

    def place(self, bng):
        bng.create_bump(self.name, self.width, self.length, self.height,
                        self.upper_length, self.upper_width, self.pos,
                        None, material=self.material, rot_quat=self.rot)


class ProceduralCone(ProceduralMesh):
    def __init__(self, pos, rot, radius, height, name,
                 rot_quat=None, material=None):
        super(ProceduralCone, self).__init__(pos, rot, name, material,
                                             rot_quat=rot_quat)
        self.radius = radius
        self.height = height

    def place(self, bng):
        bng.create_cone(self.name, self.radius, self.height, self.pos,
                        None, material=self.material, rot_quat=self.rot)


class ProceduralCube(ProceduralMesh):
    def __init__(self, pos, rot, size, name, rot_quat=None, material=None):
        super(ProceduralCube, self).__init__(pos, rot, name, material,
                                             rot_quat=rot_quat)
        self.size = size

    def place(self, bng):
        bng.create_cube(self.name, self.size, self.pos, None,
                        material=self.material, rot_quat=self.rot)


class ProceduralRing(ProceduralMesh):
    def __init__(self, pos, rot, radius, thickness, name,
                 rot_quat=None, material=None):
        super(ProceduralRing, self).__init__(pos, rot, name, material,
                                             rot_quat=rot_quat)
        self.radius = radius
        self.thickness = thickness

    def place(self, bng):
        bng.create_ring(self.name, self.radius, self.thickness, self.pos,
                        None, material=self.material, rot_quat=self.rot)
