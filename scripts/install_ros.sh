#!/bin/bash
# install_ros.sh - Instala ROS Noetic con herramientas para desarrollo de paquetes (sin Gazebo)

set -e

echo "[1/6] Configurando fuentes de ROS..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo "[2/6] Instalando ROS base (sin Gazebo)..."
sudo apt update
sudo apt install -y ros-noetic-ros-base

echo "[3/6] Instalando herramientas de desarrollo y dependencias de construcción de paquetes..."
sudo apt install -y \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool \
  python3-rosdep \
  python3-colcon-common-extensions \
  build-essential \
  python3-catkin-tools \
  python3-pip \
  python3-vcstool \
  cmake

echo "[4/6] Inicializando rosdep..."
sudo rosdep init || echo "rosdep ya inicializado."
rosdep update

echo "[5/6] Añadiendo entorno ROS al bashrc..."
if ! grep -Fxq "source /opt/ros/noetic/setup.bash" ~/.bashrc
then
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
fi
source ~/.bashrc

echo "[6/6] ROS Noetic instalado correctamente. Listo para crear workspaces."

