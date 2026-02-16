from __future__ import annotations

import uuid
from random import Random
import math


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


def lua_serialize(d) -> str:
    """
    Converts a Python object to a string that can be parsed by Lua's deserialize (loadstring) function.
    """
    return "".join(_lua_serialize_recursive(d, []))


def _lua_serialize_recursive(v, lst: list[str]) -> list[str]:
    if isinstance(v, dict):
        lst.append("{")
        for k, value in v.items():
            if not isinstance(k, str):
                raise ValueError(f"The keys must be strings, got {type(k)}")
            lst.append("[")
            lst.append(_lua_serialize_string(k))
            lst.append("]")
            lst.append("=")
            lst = _lua_serialize_recursive(value, lst)
            lst.append(",")
        if len(v) > 0:
            lst.pop()
        lst.append("}")
    elif isinstance(v, (list, tuple, set)):
        lst.append("{")
        for value in v:
            lst = _lua_serialize_recursive(value, lst)
            lst.append(",")
        if len(v) > 0:
            lst.pop()
        lst.append("}")
    elif isinstance(v, str):
        lst.append(_lua_serialize_string(v))
    elif isinstance(v, bool):
        lst.append("true" if v else "false")
    elif isinstance(v, float):
        if v == float("inf"):
            lst.append("math.huge")
        elif v == float("-inf"):
            lst.append("-math.huge")
        elif math.isnan(v):
            lst.append("0/0")
        else:
            lst.append(str(v))
    elif isinstance(v, int):
        lst.append(str(v))
    elif v is None:
        lst.append("nil")
    else:
        raise ValueError(f"Unsupported type: {type(v)}")
    return lst


def _lua_serialize_string(v: str) -> str:
    escaped = []
    for ch in v:
        if ch == "\\":
            escaped.append("\\\\")
        elif ch == "'":
            escaped.append("\\'")
        elif ch == "\n":
            escaped.append("\\n")
        elif ch == "\r":
            escaped.append("\\r")
        elif ch == "\t":
            escaped.append("\\t")
        elif ch == "\a":
            escaped.append("\\a")
        elif ch == "\b":
            escaped.append("\\b")
        elif ch == "\f":
            escaped.append("\\f")
        elif ch == "\v":
            escaped.append("\\v")
        elif ord(ch) < 32:
            escaped.append(f"\\{ord(ch):03d}")
        else:
            escaped.append(ch)
    return f"'{''.join(escaped)}'"
