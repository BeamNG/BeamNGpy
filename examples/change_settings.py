"""
This example script shows how to use the ``change_setting`` function of the
``BeamNGpy`` class to change graphics settings on startup.
"""

from beamngpy import BeamNGpy, Scenario, Vehicle


def main():
    beamng = BeamNGpy('localhost', 64256)

    with beamng as bng:
        bng.change_setting('GraphicDisplayModes', 'Fullscreen')
        bng.change_setting('GraphicDisplayResolutions', '1920 1080')
        bng.apply_graphics_setting()
        input('Press enter when done...')


if __name__ == '__main__':
    main()
