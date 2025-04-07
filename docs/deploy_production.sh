#!/bin/bash
# Deploy script for documentation pages
set -eo pipefail

# Variables
echo $DOCUMENTATION_REPOSITORY
echo $DOCUMENTATION_BRANCH
echo $DOCUMENTATION_ARTIFACTS_PATH
echo $DOCUMENTATION_TARGET_PATH

# Set up SSH
mkdir -p ~/.ssh
chmod 700 ~/.ssh
echo "$DOCUMENTATION_SSH_PRIVATE_KEY" > ~/.ssh/id_ed25519
chmod 400 ~/.ssh/id_ed25519
ls -al ~/.ssh
REPO_HOST=${DOCUMENTATION_REPOSITORY#*@}
REPO_HOST=${REPO_HOST%:*}
ssh-keyscan -t ed25519 $REPO_HOST >> ~/.ssh/known_hosts

# Clone repository
LOCAL_REPO_PATH="docs/production"
mkdir -p $LOCAL_REPO_PATH
git clone $DOCUMENTATION_REPOSITORY $LOCAL_REPO_PATH

# TODO
ls $LOCAL_REPO_PATH
