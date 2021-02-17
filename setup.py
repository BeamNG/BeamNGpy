import setuptools
import os

def read(fil):
    fil = os.path.join(os.path.dirname(__file__), fil)
    with open(fil, encoding='utf-8') as f:
        return f.read()

setuptools.setup(version=read('src/beamngpy/version.txt'))
