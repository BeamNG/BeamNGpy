#!/bin/bash
# Deploy script for documentation pages
set -eo pipefail

# Variables
echo $DOCUMENTATION_ARTIFACTS_PATH
echo $DOCUMENTATION_TARGET_PATH

# Set up SSH
mkdir -p ~/.ssh
chmod 700 ~/.ssh
echo "$DOCUMENTATION_SSH_PRIVATE_KEY" | base64 --decode > ~/.ssh/id_ed25519
chmod 600 ~/.ssh/id_ed25519
REPO_HOST=${DOCUMENTATION_REPOSITORY#*@}
REPO_HOST=${REPO_HOST%:*}
ssh-keyscan -t ed25519 $REPO_HOST >> ~/.ssh/known_hosts

# Clone repository
LOCAL_REPO_PATH="docs/production"
mkdir -p $LOCAL_REPO_PATH
git clone -b $DOCUMENTATION_BRANCH $DOCUMENTATION_REPOSITORY $LOCAL_REPO_PATH

# Git author config
git config --global user.name "$DOCUMENTATION_GIT_USERNAME"
git config --global user.email "$DOCUMENTATION_GIT_EMAIL"

# Copy artifacts
DEST_PATH="$PWD/$LOCAL_REPO_PATH/$DOCUMENTATION_TARGET_PATH"
rm -rf "$DEST_PATH"
mkdir -p "$(dirname "$DEST_PATH")"
cp -r "$DOCUMENTATION_ARTIFACTS_PATH" "$DEST_PATH"
ls -al "$DEST_PATH"

# Commit and push
cd "$LOCAL_REPO_PATH"
git add --all
git commit -m "Update $DOCUMENTATION_TARGET_PATH"
git status
git push
