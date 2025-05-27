#!/bin/bash
# install_splishsplash.sh - Installs SPlisHSPlasH with comprehensive dependency handling

set -e

echo "[1/7] Cloning SPlisHSPlasH repository..."
cd ~

if [ ! -d "SPlisHSPlasH" ]; then
  git clone https://bitbucket.org/hbpneurorobotics/splishsplash.git SPlisHSPlasH
else
  cd ~/SPlisHSPlasH
  git checkout master
  git pull
fi

cd ~/SPlisHSPlasH

echo "[2/7] Updating submodules..."
git submodule update --init --recursive

echo "[3/7] Installing core dependencies..."
sudo apt-get update
sudo apt-get install -y \
  liboctomap-dev \
  libfcl-dev \
  libassimp-dev \
  libbullet-dev \
  libeigen3-dev \
  build-essential \
  libboost-all-dev \
  freeglut3-dev \
  libxi-dev \
  liboctomap-dev \
  libxmu-dev \
  libignition-math6-dev \
  libignition-transport8-dev \
  libignition-msgs5-dev \
  libignition-common3-dev \
  libignition-fuel-tools4-dev \
  libignition-common3-graphics-dev libignition-common3-dev

echo "[4/7] Configuring DART paths..."
# Create symlinks if they don't exist
if [ ! -d "/usr/lib/cmake/dart" ]; then
  sudo mkdir -p /usr/lib/cmake
  sudo ln -s /usr/lib/x86_64-linux-gnu/cmake/dart /usr/lib/cmake/dart 2>/dev/null || true
fi

if [ ! -d "/usr/share/dart/cmake" ]; then
  sudo mkdir -p /usr/share/dart
  sudo ln -s /usr/lib/x86_64-linux-gnu/cmake/dart /usr/share/dart/cmake 2>/dev/null || true
fi

sudo ldconfig

echo "[5/7] Building SPlisHSPlasH..."
if [ ! -d "build" ]; then
  mkdir build && cd build
else
  cd build
  # Clear previous build attempts
  rm -rf *
fi
cmake \
  -DCMAKE_INSTALL_PREFIX=$HOME/.local \
  -DCMAKE_MODULE_PATH=/usr/lib/x86_64-linux-gnu/cmake/dart \
  -DCMAKE_PREFIX_PATH="/usr/lib/x86_64-linux-gnu/octomap" \
  -DCMAKE_PREFIX_PATH="/usr/local/lib/cmake"\
  -DUSE_AVX=On \
  -DUSE_OpenMP=On \
  ..

echo "[5/6] Compiling..."
make -j10

echo "[6/6] Installing SPlisHSPlasH..."
sudo make install
sudo ldconfig

echo "SPlisHSPlasH installed successfully at $HOME/.local"
cd ~
