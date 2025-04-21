#!/bin/bash
# install_gazebo.sh - Installs Gazebo from source

set -e

echo "[1/4] Cloning Gazebo repository..."
git clone https://bitbucket.org/hbpneurorobotics/gazebo.git
cd gazebo

echo "[2/4] Building Gazebo..."
mkdir build && cd build
cmake -DCMAKE_POLICY_DEFAULT_CMP0057=NEW -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_INSTALL_PREFIX=$HOME/.local ..
make -j$(nproc)

echo "[3/4] Installing Gazebo..."
sudo make install
sudo ldconfig

echo "[4/4] Gazebo installed successfully at $HOME/.local"
cd ~
