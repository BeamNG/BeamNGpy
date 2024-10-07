from __future__ import annotations

from typing import TYPE_CHECKING

from matplotlib import colors

if TYPE_CHECKING:
    from beamngpy.types import Color, Float4


def rgba_to_str(color: Float4) -> str:
    """
    Converts an ``(R, G, B, A)`` tuple of floats to a string format parsable by BeamNG.

    Returns:
        The converted string of format 'R G B A'.
    """
    return " ".join(map(str, color))


def coerce_color(color: Color, alpha=0.0) -> Float4:
    """
    Tries to coerce a color to a 4-tuple of floats.

    Args:
        color: A vehicle color.
        alpha: The alpha (transparency) value of the color. Defaults to 0.0.

    Returns:
        An ``(R, G, B, A)`` tuple of floats.
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
