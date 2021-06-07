import click
from pathlib import Path
from beamngpy import BeamNGpy


@click.group()
def cli():
    pass


@cli.command()
@click.argument('userpath', type=click.Path(file_okay=False))
def deploy_mod(userpath):
    BeamNGpy.deploy_mod(userpath)


@cli.command()
@click.argument('userpath', type=click.Path(file_okay=False))
@click.option('--host', '-h', default='localhost')
@click.option('--port', '-p', default=64256)
def setup_workspace(userpath, host, port):
    licence = Path(userpath)
    research_licence = licence / 'research.key'
    tech_licence = licence / 'tech.key'
    if not(research_licence.exists()) and not(tech_licence.exists()):
        print('Cannot set up workspace if no '
              f'licence key is available at <{userpath}>.')
        return

    bng = BeamNGpy(host, port, user=userpath)
    bng.setup_workspace()

    bng.determine_effective_userpath()
    if bng.effective_user is not None:
        print(f'Set up workspace at <{userpath}>.')
    else:
        print(f'Could not set up workspace at <{userpath}>.'
              'Note that this step is only neccessary for '
              'BeamNG.tech version 0.22 and above.')


if __name__ == '__main__':
    cli()
