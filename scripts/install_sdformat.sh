#!/bin/bash
# install_sdformat.sh - Installs sdformat9

set -e

echo "[1/4] Cloning sdformat repository..."
cd ~
git clone https://github.com/osrf/sdformat -b sdf9
cd sdformat

echo "[2/4] Building sdformat..."
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local
make -j$(nproc)

echo "[3/4] Installing sdformat..."
make install

echo "[4/4] sdformat9 installed successfully at $HOME/.local"
cd ~
