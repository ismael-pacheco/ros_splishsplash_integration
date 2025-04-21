#!/bin/bash
# install_gazebo.sh - Installs Gazebo from source

set -e

# Detect script directory to resolve relative paths
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "[1/4] Cloning Gazebo repository..."
cd ~
git clone https://bitbucket.org/hbpneurorobotics/gazebo.git
cd gazebo

echo "[2/4] Building Gazebo..."
mkdir build && cd build
cmake -DCMAKE_POLICY_DEFAULT_CMP0057=NEW -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_INSTALL_PREFIX=$HOME/.local ..
make -j10

echo "[3/4] Installing Gazebo..."
sudo make install
sudo ldconfig

echo "[4/4] Copying test files to ~/gazebo..."
cp -r "$REPO_ROOT/test files/"* "$HOME/gazebo/"

echo "Gazebo installed successfully at $HOME/.local"
cd ~

