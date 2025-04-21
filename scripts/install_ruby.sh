#!/bin/bash
# install_ruby.sh - Configures Ruby and RVM

set -e

echo "[1/4] Importing GPG keys..."
gpg --keyserver hkp://keyserver.ubuntu.com --recv-keys 409B6B1796C275462A1703113804BB82D39DC0E3 7D2BAF1CF37B13E2069D6956105BD0E739499BDB

echo "[2/4] Installing RVM..."
curl -sSL https://get.rvm.io | bash -s stable

echo "[3/4] Setting up RVM environment..."
source ~/.rvm/scripts/rvm

echo "[4/4] Installing Ruby 1.9.3..."
rvm install 1.9.3
rvm use 1.9.3 --default
