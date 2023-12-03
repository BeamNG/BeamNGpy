from __future__ import annotations

from logging import DEBUG, getLogger

from beamngpy.logging import LOGGER_ID

from beamngpy import BeamNGpy


# The Terrain Importer class.
class Terrain_Importer:

    @staticmethod
    def import_heightmap(bng: BeamNGpy, data, w, h, scale = 1, zMin = 0, zMax = 100, isYFlipped = True):
        """
        Imports a heightmap from a 2D structure of 'pixel' values in the range given by [zMin, zMax].  The array should be indexable as [x][y].
        The scale describes the resolution of the final generated heightmap - in both x and y, in metres-per-pixel. The scale is isotropic.
        Heightmaps must ultimately satisfy the following conditions, which this function will ensure (by cropping, if required):
            i)    They must be square, ie the width and height must be the same.
            ii)   They must be a power of two in size, eg 2^4, 2^5, 2^6 etc.
            iii)  They must be between 128 and 8192 in length. This is the minimum and maximum possible heightmap data size.
        The generated heightmap will assume the vertical range [0, zMax - zMin] to ensure maximum granularity in its height.

        Args:
            bng: The BeamNG instance.
            data: A 2D dict of heightmap pixel values, indexed as [x][y].
            w: The width of the data (X dimension).
            h: The height of the data (Y dimension).
            scale: The scale of the data/resultant heightmap, in metres-per-pixel.
            zMin: The smallest elevation value in the data, in metres.
            zMax: The largest elevation value in the data, in metres.
            isYFlipped: A flag which indicates if the heightmap should be flipped in the Y dimension.
        """
        logger = getLogger(f'{LOGGER_ID}.Terrain_Importer')
        logger.setLevel(DEBUG)
        d = dict(type = 'ImportHeightmap', data = data, w = w, h = h, scale = scale, zMin = zMin, zMax = zMax, isYFlipped = isYFlipped)
        response = bng.connection.send(d)
        response.ack('CompletedImportHeightmap')
        logger.debug('Terrain_Importer - terrain imported.')