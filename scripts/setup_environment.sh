#!/bin/bash
# setup_environment.sh - Configures environment variables

echo "[1/2] Setting up environment variables..."
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc

echo "[2/2] Sourcing bashrc..."
source ~/.bashrc
echo "Environment configured successfully"
set -e
BASE_DIR=~/gazebo
WS=$BASE_DIR/catkin_ws
PX4_DIR=~/PX4-Autopilot

echo "[INFO] Creando directorios de destino..."

mkdir -p "$WS/src/iq_gnc/scripts"
mkdir -p "$WS/src/iq_gnc/src"
mkdir -p "$PX4_DIR/launch"

echo "[INFO] Copiando escenarios Gazebo..."
cp test\\ files/gazebo/*.sdf "$BASE_DIR/"

echo "[INFO] Copiando scripts de IQ_GNC..."
cp -r test\\ files/iq_gnc/scripts/* "$WS/src/iq_gnc/scripts/"

echo "[INFO] Copiando código fuente de IQ_GNC..."
cp -r test\\ files/iq_gnc/src/* "$WS/src/iq_gnc/src/"

echo "[INFO] Copiando launch files de PX4-Autopilot..."
cp test\\ files/PX4-Autopilot/launch/*.launch "$PX4_DIR/launch/"

echo "[INFO] Setup del entorno completado."
