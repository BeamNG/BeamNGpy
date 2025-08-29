from __future__ import annotations

def get_sensor_name(prefix: str | None, name: str | None, default_name: str) -> str:
    if prefix == None:
        prefix = ""
    if name == None:
        name = default_name
    if not prefix:
        return name
    if name.startswith(prefix + "_"):
        return name
    return prefix + "_" + name
