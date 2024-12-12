from __future__ import annotations

from logging import DEBUG, getLogger
from typing import Any, cast

from beamngpy.logging import LOGGER_ID
from beamngpy.types import Int2, StrDict

module_logger = getLogger(f"{LOGGER_ID}.level")
module_logger.setLevel(DEBUG)


class Level:
    """
    Represents a level in the simulator, listing various properties like the
    level's name, size, and available scenarios.
    """

    @staticmethod
    def from_dict(d: StrDict) -> Level:
        if "levelName" in d:
            name = d["levelName"]
            del d["levelName"]
        else:
            name = "unknown"

        if "size" in d:
            assert len(d["size"]) == 2
            size = (int(d["size"][0]), int(d["size"][1]))
            del d["size"]
        else:
            size = (-1, -1)

        if "misFilePath" in d:
            path = d["misFilePath"]
            if path[0] == "/":
                #  Drop leading / in path
                path = path[1:]
            del d["misFilePath"]
        else:
            path = None

        level = Level(name, size, path, **d)

        return level

    def __init__(self, name: str, size: Int2, path: str | None, **props: Any):
        self.logger = getLogger(f"{LOGGER_ID}.Level")
        self.logger.setLevel(DEBUG)
        self.name = name
        self.size = size
        self.path = path
        if path is None:
            # Use object id as path to make it unique when none is given
            path = "unknown.{}".format(id(self))
            self.logger.debug(f"No path given, setting path to '{path}'")

        self.properties = props
        prop_description = ", ".join(list(self.properties.keys()))
        self.logger.debug(
            "adding these properties to level object: " f"{prop_description}"
        )

    def __str__(self) -> str:
        return self.name

    def __hash__(self) -> int:
        return hash(self.path)

    def __eq__(self, o: Any) -> bool:
        if isinstance(o, Level):
            return self.path == o.path
        return False
