#!/bin/bash
# Deploy script for documentation pages
set -eo pipefail

# Variables
echo $DOCUMENTATION_REPOSITORY
echo $DOCUMENTATION_BRANCH
echo $DOCUMENTATION_GIT_USERNAME
echo $DOCUMENTATION_GIT_EMAIL
echo $DOCUMENTATION_ARTIFACTS_PATH
echo $DOCUMENTATION_TARGET_PATH

# Set up SSH
mkdir -p ~/.ssh
echo $DOCUMENTATION_SSH_PRIVATE_KEY > ~/.ssh/id_ed25519
chmod 600 ~/.ssh/id_ed25519
ssh-keyscan -t ed25519 $DOCUMENTATION_REPOSITORY >> ~/.ssh/known_hosts

# Clone repository
LOCAL_REPO_PATH="docs/production"
mkdir -p $LOCAL_REPO_PATH
git clone $DOCUMENTATION_REPOSITORY $LOCAL_REPO_PATH

# TODO
ls $LOCAL_REPO_PATH
