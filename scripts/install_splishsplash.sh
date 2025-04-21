#!/bin/bash
# install_splishsplash.sh - Installs SPlisHSPlasH

set -e

echo "[1/5] Cloning SPlisHSPlasH repository..."
git clone https://bitbucket.org/hbpneurorobotics/splishsplash.git SPlisHSPlasH
cd SPlisHSPlasH

echo "[2/5] Updating submodules..."
git submodule update --init --recursive

echo "[3/5] Building SPlisHSPlasH..."
mkdir build && cd build
cmake -DCMAKE_POLICY_DEFAULT_CMP0000=OLD -DCMAKE_POLICY_DEFAULT_CMP0057=NEW -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_INSTALL_PREFIX=$HOME/.local -DENABLE_CUDA=ON ..
make -j$(nproc)

echo "[4/5] Installing SPlisHSPlasH..."
sudo make install
sudo ldconfig

echo "[5/5] SPlisHSPlasH installed successfully at $HOME/.local"
cd ~
