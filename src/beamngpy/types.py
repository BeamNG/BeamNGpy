from __future__ import annotations

from typing import Any, Dict, Tuple, Union

# these empty comments are because of the autodocumentation

StrDict = Dict[str, Any]
""
Float2 = Tuple[float, float]
""
Float3 = Tuple[float, float, float]
""
Float4 = Tuple[float, float, float, float]
""
Float5 = Tuple[float, float, float, float, float]
""
Int2 = Tuple[int, int]
""
Int3 = Tuple[int, int, int]
""
Quat = Tuple[float, float, float, float]
""
Color = Union[Float3, Float4, str]
"""
Vehicle color. Can be either:

    - ``(R, G, B)`` tuple of floats between 0.0 and 1.0,
    - ``(R, G, B, A)`` tuple of floats between 0.0 and 1.0,
    - string of format ``'R G B'``, where ``R``, ``G``, and ``B`` are floats between 0.0 and 1.0,
    - string of format ``'R G B A'``, where ``R``, ``G``, ``B``, and ``A`` are floats between 0.0 and 1.0,
    - a common color name (parsable by ``matplotlib.colors``).
"""
Time = Union[float, str]
