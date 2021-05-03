import click

from beamngpy import BeamNGpy


@click.group()
def cli():
    pass


@cli.command()
@click.argument('userpath', type=click.Path(file_okay=False))
def deploy_mod(userpath):
    BeamNGpy.deploy_mod(userpath)

@cli.command()
@click.option('--host', '-h', type=str)
@click.option('--port', '-p', type=str)
@click.option('--userpath', '-u', type=click.Path(file_okay=False))
def setup_workspace(host, port, userpath):
    licence = Path(userpath)
    research_licence = licence / 'research.key'
    tech_licence = licence / 'tech.key'
    if not(research_licence.exists()) and not(tech_licence.exists()):
        print('Cannot set up workspace if no '
              f'licence key is available at <{userpath}>.')
        return
    bng = BeamNGpy(host, port, user=userpath)
    bng.open(deploy=False, lua='exit(0)')
    researchHelper = Path(userpath)/'researchHelper.txt'
    if researchHelper.exists():
        print(f'Set up workspace at <{userpath}>.')
    else:
        print(f'Could not set up workspace at <{userpath}>.'
              'Note that this step is only neccessary for'
              'BeamNG.tech version 0.23 and above.')


if __name__ == '__main__':
    cli()
