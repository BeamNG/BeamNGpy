import setuptools

import os
import os.path

import site
import sys

site.ENABLE_USER_SITE = "--user" in sys.argv[1:]


def long_description():
    here = os.path.abspath(os.path.dirname(__file__))
    with open(os.path.join(here, "README.md")) as infile:
        return infile.read()


def read(fil):
    fil = os.path.join(os.path.dirname(__file__), fil)
    with open(fil, encoding="utf-8") as f:
        return f.read()


setuptools.setup(
    version=read("src/beamngpy/version.txt"),
    long_description=long_description(),
    long_description_content_type="text/markdown",
)
