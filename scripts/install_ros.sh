#!/bin/bash
# install_ros.sh - Instala ROS Noetic sin Gazebo, iq_sim, Ardupilot y su plugin para Gazebo

set -e
echo "[INFO] Instalando ROS Noetic (sin Gazebo)..."
sudo apt update
sudo apt install -y curl gnupg2 lsb-release
# Añadir repositorio ROS
sudo sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros1-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
# Instalar ROS base (sin gazebo)
sudo apt install -y ros-noetic-ros-base python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# Inicializar rosdep
sudo rosdep init || true
rosdep update

echo "[INFO] Instalando iq_sim y dependencias..."
# iq_sim depende de estos paquetes
sudo apt install -y ros-noetic-joy ros-noetic-xacro ros-noetic-gazebo-plugins

echo "[INFO] Clonando e instalando ArduPilot y plugin Gazebo..."
# ArduPilot
ARDUPILOT_DIR=~/ardupilot
if [ -d \"$ARDUPILOT_DIR\" ]; then
  cd \"$ARDUPILOT_DIR\" && git pull
else
  git clone https://github.com/ArduPilot/ardupilot.git \"$ARDUPILOT_DIR\"
  cd \"$ARDUPILOT_DIR\"
fi
# Plugin Gazebo para ArduPilot
cd Tools/sitl_gazebo
git submodule update --init --recursive
./setup_gazebo.sh
pip3 install --user -r python_requirements.txt

echo "[INFO] Configuración de entorno..."
cat <<EOF >> ~/.bashrc

# ROS Noetic
source /opt/ros/noetic/setup.bash
# IQ_Sim y ArduPilot SITL
export PATH=\"\$PATH:$ARDUPILOT_DIR/Tools/autotest\"
export GAZEBO_PLUGIN_PATH=\"$ARDUPILOT_DIR/Tools/sitl_gazebo/build\"\nEOF

source ~/.bashrc
echo "[INFO] Instalación ROS e IQ_Sim completada."

