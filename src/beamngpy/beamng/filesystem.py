from __future__ import annotations

import logging
import os
import platform
from pathlib import Path

from beamngpy.logging import LOGGER_ID, BNGError, BNGValueError

BINARIES = [
    "Bin64/BeamNG.tech.x64.exe",
    "Bin64/BeamNG.x64.exe",
    "Bin64/BeamNG.drive.x64.exe",
]
BINARIES_LINUX = [
    "BinLinux/BeamNG.tech.x64",
    "BinLinux/BeamNG.x64",
    "BinLinux/BeamNG.drive.x64",
]

logger = logging.getLogger(f"{LOGGER_ID}.BeamNGpy")
logger.setLevel(logging.DEBUG)


def determine_home(home: str | None) -> Path:
    if not home:
        home = os.getenv("BNG_HOME")
    if not home:
        raise BNGValueError(
            "No BeamNG home folder given. Either specify "
            "one in the constructor of `BeamNGpy` or define an "
            'environment variable "BNG_HOME" that '
            "points to where your copy of BeamNG.* is."
        )

    return Path(home).resolve()


def determine_userpath(binary: Path) -> Path | None:
    """
    Tries to find the userpath based on the beamng installation if the user
    did not provide a custom userpath.
    """
    if platform.system() == "Linux":
        return None
    user = Path.home() / "AppData"
    user = user / "Local"
    if ".research" in binary.name:
        user = user / "BeamNG.research"
    elif ".tech" in binary.name:
        user = user / "BeamNG.tech"
    else:
        user = user / "BeamNG.drive"
    logger.debug(f"Userpath is set to {user.as_posix()}")
    return user


def determine_binary(home: Path) -> Path:
    """
    Tries to find one of the common BeamNG-binaries in the specified home
    path and returns the discovered path as a string.

    Returns:
        Path to the binary as a string.

    Raises:
        BNGError: If no binary could be determined.
    """
    choice = None
    binaries = BINARIES_LINUX if platform.system() == "Linux" else BINARIES
    for option in binaries:
        binary = home / option
        if binary.exists():
            choice = binary
            break

    if not choice:
        raise BNGError(
            "No BeamNG binary found in BeamNG home. Make "
            "sure any of these exist in the BeamNG home "
            f'folder: {", ".join(binaries)}'
        )

    logger.debug(f"Determined BeamNG.* binary to be: {choice}")
    return choice
