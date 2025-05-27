#!/bin/bash
# install_ignition_math.sh - Installs ign-math from source (sin PPA)

set -e

echo "[1/4] Removing existing ign-math packages..."
sudo apt-get remove -y 'libignition-math*' || true

echo "[2/4] Cloning ign-math repository..."
cd ~
if [ ! -d "ign-math" ]; then
  git clone https://github.com/gazebosim/ign-math -b ign-math6
else
  cd ign-math
  git checkout ign-math6
  git pull
  cd ~
fi

echo "[3/4] Building ign-math..."
cd ign-math
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j$(nproc)

echo "[4/4] Installing ign-math..."
sudo make install
sudo ldconfig
cd ~
