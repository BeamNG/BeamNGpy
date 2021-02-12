import click

from beamngpy import BeamNGpy


@click.group()
def cli():
    pass


@cli.command()
@click.argument('userpath', type=click.Path(file_okay=False))
def deploy_mod(userpath):
    BeamNGpy.deploy_mod(userpath)


if __name__ == '__main__':
    cli()
