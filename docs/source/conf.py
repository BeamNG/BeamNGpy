# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys

sys.path.insert(0, os.path.abspath("../../src"))

# -- Project information -----------------------------------------------------

project = "BeamNGpy"
copyright = "2025, BeamNG GmbH"
author = "BeamNG GmbH"


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.extlinks",
    "sphinx_rtd_theme",
    "m2r2",
    "sphinx_multiversion",
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

html_favicon = "favicon.ico"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]


# source_suffix = '.rst'
source_suffix = [".rst", ".md"]

# -- Napoleon options --------------------------------------------------------
napoleon_google_docstring = True

# -- Autodoc options ---------------------------------------------------------
autodoc_mock_imports = ["msgpack", "OpenGL", "PIL", "matplotlib", "numpy", "seaborn"]
autodoc_typehints = "both"
autodoc_type_aliases = {
    "StrDict": "StrDict",
    "Float2": "Float2",
    "Float3": "Float3",
    "Float4": "Float4",
    "Float5": "Float5",
    "Int2": "Int2",
    "Int3": "Int3",
    "Quat": "Quat",
    "Color": "Color",
}

# -- Extlinks options --------------------------------------------------------
extlinks = {
    "pydocs": (
        "beamngpy.html#%s",
        "%s",
    ),
    "repo132": ("https://github.com/BeamNG/BeamNGpy/blob/v1.32/%s", "%s"),
    "repo133": ("https://github.com/BeamNG/BeamNGpy/blob/v1.33/%s", "%s"),
    "blog": ("https://beamng.tech/blog/%s", None),
    "techdocs": ("https://docs.beamng.com/beamng_tech/%s", None),
    "drivedocs": ("https://docs.beamng.com/%s", None),
}

# -- Multi-version config ----------------------------------------------------
smv_tag_whitelist = r"^v.*$"
smv_branch_whitelist = r"^(master|dev)$"
