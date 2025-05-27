#!/bin/bash
# install_gazebo.sh - Installs Gazebo from source

set -e

# Detect script directory to resolve relative paths
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "[1/4] Cloning Gazebo repository..."
cd ~
sudo apt update && sudo apt install -y \
    libprotobuf-dev protobuf-compiler \
    libignition-math6-dev \
    libignition-transport8-dev \
    libignition-common3-dev \
    libignition-fuel-tools4-dev \
    libignition-msgs5-dev \
    libsimbody-dev \
    libfreeimage-dev \
    libgts-dev \
    libavdevice-dev \
    libgraphviz-dev

if [ ! -d "gazebo" ]; then
  git clone https://bitbucket.org/hbpneurorobotics/gazebo.git
else
  cd ~/gazebo
  git checkout master
  git pull

fi
cd ~/gazebo


# ✨ Inject C++14 requirement into Gazebo's root CMakeLists.txt
echo "[INFO] Forcing C++14 standard in Gazebo CMakeLists.txt..."
sed -i '1iset(CMAKE_CXX_STANDARD 14)\nset(CMAKE_CXX_STANDARD_REQUIRED ON)' CMakeLists.txt

echo "[2/4] Building Gazebo..."

if [ ! -d "build" ]; then
  mkdir build && cd build
else
  cd build
fi

cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local ..

make -j10 2> errors.log

echo "[3/4] Installing Gazebo..."
sudo make install
sudo ldconfig

echo "[4/4] Copying test files to ~/gazebo..."
cp -r "$REPO_ROOT/test files/"* "$HOME/gazebo/"

echo "Gazebo installed successfully at $HOME/.local"
cd ~

