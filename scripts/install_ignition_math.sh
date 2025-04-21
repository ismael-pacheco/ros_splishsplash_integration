#!/bin/bash
# install_ignition_math.sh - Installs ign-math from source

set -e

echo "[1/5] Adding Ignition Robotics PPA..."
sudo apt-add-repository ppa:ignitionrobotics/ppa -y
sudo apt-get update

echo "[2/5] Removing existing ign-math packages..."
sudo apt-get remove -y libignition-math6-dev

echo "[3/5] Cloning ign-math repository..."
git clone https://github.com/ignitionrobotics/ign-math -b ign-math6
cd ign-math

echo "[4/5] Building ign-math..."
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j$(nproc)

echo "[5/5] Installing ign-math..."
sudo make install
sudo ldconfig
cd ~
