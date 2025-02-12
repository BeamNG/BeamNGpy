#!/bin/bash
# Build script for documentation pages
set -eo pipefail

# Go to this directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR"

# Add relevant branches locally
git fetch origin master:master
if [ "$USE_DEV_BRANCH" == "true" ]; then
    git fetch origin docs-multiversion:dev
fi

# Build docs
pip install -r source/requirements.txt
sphinx-multiversion source build
cp root_template.html build/index.html
if [ "$USE_DEV_BRANCH" == "true" ]; then
    cp root_template_dev.html build/index.html
fi
