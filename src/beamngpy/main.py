import logging as log
import os
import shutil
import sys

from time import sleep

import click

from . import beamng
from . import bnpcfg

DEF_LOG = 'beamngpy.log'
DEF_CFG = "beamngpy.cfg"


def log_exception(extype, value, trace):
    """
    Hook to log uncaught exceptions to the logging framework. Register this as
    the excepthook with `sys.excepthook = log_exception`.
    """
    log.exception("Uncaught exception: ", exc_info=(extype, value, trace))


def setup_logging(log_file):
    """
    Sets up the logging framework to log to the given log_file and to STDOUT.
    If the path to the log_file does not exist, directories for it will be
    created.
    """
    if os.path.exists(log_file):
        backup = f'{log_file}.1'
        shutil.move(log_file, backup)

    file_handler = log.FileHandler(log_file, "w", "utf-8")
    term_handler = log.StreamHandler()
    handlers = [term_handler, file_handler]
    fmt = "%(asctime)s %(levelname)-8s %(message)s"
    log.basicConfig(handlers=handlers, format=fmt, level=log.DEBUG)

    sys.excepthook = log_exception

    log.info("Started BeamNGPy logging to: %s", log_file)


@click.group()
@click.option("--log-file", type=click.Path(dir_okay=False), default=DEF_LOG)
@click.option("--cfg-file", type=click.Path(dir_okay=False), default=DEF_CFG)
def cli(log_file=None, cfg_file=None):
    """
    Click group that ensures at least log and configuration file are present,
    since the rest of the application uses those.

    :param log_file:
        The file to log to. An existing file will be backed up to {log_file}.1
    :param cfg_file:
        Config file to read.
    """
    setup_logging(log_file)
    bnpcfg.ensure_config(cfg_file)


@cli.command()
def test_beamnpy():
    with beamng.BeamNPy(bnpcfg.CFG.ipc_host, bnpcfg.CFG.ipc_port) as bpy:
        while True:
            print(bpy.recv_one())


if __name__ == "__main__":
    cli()
