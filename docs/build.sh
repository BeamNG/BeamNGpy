#!/bin/bash
# Build script for documentation pages
set -eo pipefail

# Go to this directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR"

# Add relevant branches locally
git fetch origin master:master
if [ "$USE_DEV_BRANCH" == "true" ]; then
    git fetch origin dev:dev
    BRANCH="dev"
else
    BRANCH="master"
fi

# Build docs
pip install -r source/requirements.txt
sphinx-multiversion source build
python generate_index.py $BRANCH build/index.html
