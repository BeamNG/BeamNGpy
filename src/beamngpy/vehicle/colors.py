from __future__ import annotations

from matplotlib import colors

from beamngpy.types import Color, Float4


def rgba_to_str(color: Float4) -> str:
    return ' '.join(map(str, color))


def coerce_vehicle_color(color: Color, alpha=0.0) -> Float4:
    """
    Tries to coerce a vehicle color to a format parsable by BeamNG.

    Args:
        color: A vehicle color.
        alpha: The alpha (transparency) value of the color. Defaults to 0.0.
    """
    if isinstance(color, str):
        try:
            color = tuple(float(x) for x in color.split())  # string of format 'R G B A'
        except ValueError:
            pass
    if isinstance(color, tuple):
        if len(color) == 3:
            return (*color, alpha)
        return color

    return colors.to_rgba(color, alpha=alpha)
