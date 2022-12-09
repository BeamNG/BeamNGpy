from __future__ import annotations

from typing import Any, Dict, Tuple, Union

# these empty comments are because of the autodocumentation

StrDict = Dict[str, Any]
''
Float2 = Tuple[float, float]
''
Float3 = Tuple[float, float, float]
''
Float4 = Tuple[float, float, float, float]
''
Float5 = Tuple[float, float, float, float, float]
''
Int2 = Tuple[int, int]
''
Int3 = Tuple[int, int, int]
''
Quat = Tuple[float, float, float, float]
''
Color = Union[Float3, Float4, str]
'''
Vehicle color. Can be either:
    - (R, G, B) tuple of floats
    - (R, G, B, A) tuple of floats
    - string of format 'R G B'
    - string of format 'R G B A'
    - a common color name
'''
