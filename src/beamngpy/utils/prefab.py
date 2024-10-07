from __future__ import annotations

import uuid
from random import Random


def get_uuid(seed: str) -> str:
    """
    Generates a UUID which can be used as an identifier for the scene objects.
    Is deterministic based on the value of the ``seed`` argument provided.

    Args:
        seed: The seed for the random generator.
    """
    rnd = Random(seed)
    return str(uuid.UUID(int=rnd.getrandbits(128), version=4))


def bool_to_str(x: bool) -> str:
    """
    Returns a string representation of a boolean that is valid in the context
    of a BeamNG json prefab. Used as a filter for the Jinja template.
    """
    return "true" if x else "false"
