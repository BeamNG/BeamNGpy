#!/bin/bash
# Script to run tests on CI
set -eo pipefail
export BNG_HOME="$PWD/$BNG_HOME"

echo "Environment variables:"
echo "Working directory: $PWD"
echo "TESTING_REVISION: $TESTING_REVISION"
echo "BNG_HOME: $BNG_HOME"
echo "HEADLESS: $HEADLESS"
echo "GFX: $GFX"

echo "Setting up Python environment ..."
rm -rf .venv
python -m venv .venv
source .venv/bin/activate
pip install -e .
pip install pytest

echo "Cleaning up BeamNG user folder ..."
rm -rf ~/.local/share/BeamNG

echo "Running tests ..."
pytest tests/test_minimal.py
