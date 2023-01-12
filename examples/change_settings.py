'''
This example script shows how to use the ``settings.change`` function of the
``BeamNGpy`` class to change graphics settings on startup.
'''

from beamngpy import BeamNGpy


def main():
    beamng = BeamNGpy('localhost', 64256)

    with beamng as bng:
        bng.settings.change('GraphicDisplayModes', 'Fullscreen')
        bng.settings.change('GraphicDisplayResolutions', '1920 1080')
        bng.settings.apply_graphics()
        input('Press enter when done...')


if __name__ == '__main__':
    main()
