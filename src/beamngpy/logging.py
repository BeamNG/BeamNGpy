from __future__ import annotations

import io
import logging
import pydoc
import warnings
from pathlib import Path
from shutil import move
from typing import Any, List

LOGGER_ID = "beamngpy"
LOG_FORMAT = "%(asctime)-24s|%(levelname)-9s|%(name)-30s|%(message)s"
bngpy_logger = logging.getLogger(LOGGER_ID)
module_logger = logging.getLogger(f"{LOGGER_ID}.beamngpycommon")
comm_logger = logging.getLogger(f"{LOGGER_ID}.communication")
bngpy_handlers = list()


class BNGError(Exception):
    """
    Generic BeamNG error.
    """

    pass


class BNGValueError(ValueError):
    """
    Value error specific to BeamNGpy.
    """

    pass


class BNGDisconnectedError(ValueError):
    """
    Exception class for BeamNGpy being disconnected when it shouldn't.
    """

    pass


def create_warning(msg: str, category: Any = None) -> None:
    """
    Helper function for BeamNGpy modules to create warnings.

    Args:
        msg: message to be displayed
        category: Category of warning to be issued. See `warnings` documentation for more details. Defaults to None.
    """
    warnings.warn(msg, category=category, stacklevel=2)


def config_logging(
    handlers: List[logging.Handler],
    replace: bool = True,
    level: int = logging.DEBUG,
    redirect_warnings: bool = True,
    log_communication: bool = False,
) -> None:
    """
    Function to configure logging.

    Args:
        handlers: list of already configured logging.Handler objects
        replace: whether to replace existing list of handlers with new ones or whether to add them, optional
        level: log level of the beamngpy logger object, optional. Defaults to ``logging.DEBUG``.
        redirect_warnings: whether to redirect warnings to the logger. Beware that this modifies the warnings settings.
        log_communication: whether to log the BeamNGpy protocol messages between BeamNGpy and BeamNG.tech, optional
    """
    global bngpy_logger, bngpy_handlers
    root_logger = logging.getLogger()
    if replace and bngpy_handlers:
        for h in bngpy_handlers:
            root_logger.removeHandler(h)
    for h in handlers:
        root_logger.addHandler(h)
    bngpy_handlers = handlers

    bngpy_logger.setLevel(level)
    comm_logger.setLevel(logging.DEBUG)
    comm_logger.disabled = not log_communication

    if redirect_warnings:
        logging.captureWarnings(redirect_warnings)
        warn_log = logging.getLogger("py.warnings")
        warnings.simplefilter("once")
        for h in handlers:
            warn_log.addHandler(h)
    bngpy_logger.info("Started BeamNGpy logging.")
    for h in handlers:
        if isinstance(h, logging.FileHandler):
            module_logger.info(f"Logging to file: {h.baseFilename}.")


def set_up_simple_logging(
    log_file: str | None = None,
    redirect_warnings: bool = True,
    level: int = logging.INFO,
    log_communication: bool = False,
) -> None:
    """
    Helper function that provides high-level control
    over beamng logging. For low-level control over the
    logging system use :func:`config_logging`.
    Sets up logging to ``sys.stderr`` and optionally to a given file.
    Existing log files are moved to ``<log_file>.1``.
    By default beamngpy logs warnings and errors to ``sys.stderr``,
    so this function is only of use, if the log output should additionaly
    be written to a file, or if the log level needs to be adjusted.

    Args:
        log_file: log filename, optional
        redirect_warnings: Whether to redirect warnings to the logger. Beware that this modifies the warnings settings.
        level: log level of handler that is created for the log file. Defaults to ``logging.INFO``.
        log_communication: whether to log the BeamNGpy protocol messages between BeamNGpy and BeamNG.tech, optional
    """
    sh = logging.StreamHandler()
    sh.setLevel(level)
    formatter = logging.Formatter(LOG_FORMAT)
    sh.setFormatter(formatter)
    handlers: List[logging.Handler] = [sh]
    moved_log = False
    fh = None
    if log_file:
        if Path(log_file).exists():
            move(log_file, f"{log_file}.1")
            moved_log = True
        fh = logging.FileHandler(log_file, "w", "utf-8")
        formatter = logging.Formatter(LOG_FORMAT)
        fh.setFormatter(formatter)
        fh.setLevel(level)
        handlers.append(fh)
    config_logging(
        handlers,
        level=level,
        redirect_warnings=redirect_warnings,
        log_communication=log_communication,
    )
    if moved_log and fh is not None:
        module_logger.info(f"Moved old log file to '{fh.baseFilename}.1'.")


def _generate_docstring(obj: Any) -> str:
    try:
        buffer = io.StringIO()
        pydoc.doc(obj, output=buffer)
        buffer.seek(0)
        docstring = buffer.read().split("\n")
        return "\n".join(docstring[2:])
    except:
        return ""
