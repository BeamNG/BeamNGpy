import click
from logging import getLogger, DEBUG
from beamngpy.beamngcommon import LOGGER_ID, create_warning
import warnings


module_logger = getLogger(f'{LOGGER_ID}.main')
module_logger.setLevel(DEBUG)


@click.group()
def cli():
    pass


@cli.command()
@click.argument('userpath', type=click.Path(file_okay=False))
def deploy_mod(userpath):
    # to show the deprecation warnings when running CLI commands
    warnings.simplefilter('always', DeprecationWarning)
    create_warning('`deploy-mod` command is not used and deprecated since BeamNG.tech version 0.25.',
        DeprecationWarning)


@cli.command()
@click.argument('userpath', type=click.Path(file_okay=False))
@click.option('--host', '-h', default='localhost')
@click.option('--port', '-p', default=64256)
def setup_workspace(userpath, host, port):
    # to show the deprecation warnings when running CLI commands
    warnings.simplefilter('always', DeprecationWarning)
    create_warning('`setup-workspace` command is not needed and deprecated since BeamNG.tech version 0.25.',
        DeprecationWarning)


if __name__ == '__main__':
    cli()
