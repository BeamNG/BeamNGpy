"""
.. module:: beamng
    :platform: Windows
    :synopsis: Contains level-related classes.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
"""


class Level:
    """
    Represents a level in the simulator, listing various properties like the
    level's name, size, and available scenarios.
    """

    @staticmethod
    def from_dict(d):
        if 'levelName' in d:
            name = d['levelName']
            del d['levelName']
        else:
            name = 'unknown'

        if 'size' in d:
            size = d['size']
            del d['size']
        else:
            size = [-1, -1]

        if 'misFilePath' in d:
            path = d['misFilePath']
            if path[0] == '/':
                #  Drop leading / in path
                path = path[1:]
            del d['misFilePath']
        else:
            path = None

        level = Level(name, size, path, **d)

        return level

    def __init__(self, name, size, path, **props):
        self.name = name
        self.size = size
        self.path = path
        if path is None:
            # Use object id as path to make it unique when none is given
            path = 'unknown.{}'.format(id(self))

        self.properties = {**props}
        self.scenarios = {}

    def __str__(self):
        return self.name

    def __hash__(self):
        return hash(self.path)

    def __eq__(self, o):
        if isinstance(o, Level):
            return self.path == o.path
        return False
