from __future__ import annotations

from beamngpy.logging import BNGValueError


def validate_object_name(name: str) -> None:
    if not name:
        raise BNGValueError("Name cannot be empty or None!")
    starting_character = name[0]
    if starting_character == "%":
        raise BNGValueError(f"Object name cannot start with a '%' - '{name}'")
    if starting_character.isdigit():
        raise BNGValueError(f"Object name cannot start with a digit - '{name}'")
    if "/" in name:
        raise BNGValueError(f"Object name cannot contain a '/'- '{name}'")
