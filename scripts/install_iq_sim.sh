#!/bin/bash
# install_iq_sim_and_ardupilot_plugin.sh
# Instala iq_sim y el plugin de Gazebo para ArduPilot (sitl_gazebo)

set -e
echo "[INFO] Instalando dependencias de iq_sim..."
# Dependencias básicas para iq_sim (asume ROS Noetic ya instalado)
sudo apt update
sudo apt install -y \
  ros-noetic-joy \
  ros-noetic-xacro \
  ros-noetic-gazebo-plugins \
  ros-noetic-mavlink \
  ros-noetic-mavros-msgs

echo "[INFO] Clonando iq_sim..."
IQ_SIM_DIR=~/iq_sim
if [ -d "$IQ_SIM_DIR" ]; then
  echo "[INFO] iq_sim ya existe en $IQ_SIM_DIR, actualizando repositorio..."
  cd "$IQ_SIM_DIR" && git pull
else
  git clone https://github.com/Intelligent-Quads/iq_sim.git "$IQ_SIM_DIR"
  cd "$IQ_SIM_DIR"
fi

echo "[INFO] Construyendo iq_sim..."
# Si usa catkin_ws, copiar iq_sim a src y compilar en workspace
WS=~/catkin_ws
mkdir -p $WS/src
cp -r "$IQ_SIM_DIR" "$WS/src/"
cd $WS
rosdep update
rosdep install --from-paths src -i -y
catkin_make

echo "[INFO] Configurando entorno de iq_sim..."
if ! grep -Fxq "source $WS/devel/setup.bash" ~/.bashrc; then
  echo "source $WS/devel/setup.bash" >> ~/.bashrc
fi

echo "[INFO] Instalando ArduPilot SITL y plugin Gazebo..."
# Clona ArduPilot (si no está) y prepara su plugin
ARDUPILOT_DIR=~/ardupilot
if [ -d "$ARDUPILOT_DIR" ]; then
  echo "[INFO] ArduPilot ya existe en $ARDUPILOT_DIR, actualizando..."
  cd "$ARDUPILOT_DIR" && git pull
else
  git clone https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
  cd "$ARDUPILOT_DIR"
fi

# Inicializa submódulos y compila solo el plugin sitl_gazebo
cd Tools/sitl_gazebo
echo "[INFO] Inicializando y actualizando submódulos de sitl_gazebo..."
git submodule update --init --recursive

echo "[INFO] Compilando plugin Gazebo para ArduPilot..."
mkdir -p build && cd build
cmake ..
make -j$(nproc)

echo "[INFO] Instalando requisitos Python del plugin..."
pip3 install --user -r ../python_requirements.txt

echo "[INFO] Añadiendo plugin al GAZEBO_PLUGIN_PATH..."
PLUG_PATH="$ARDUPILOT_DIR/Tools/sitl_gazebo/build"
if ! grep -Fxq "export GAZEBO_PLUGIN_PATH=\"$PLUG_PATH\"" ~/.bashrc; then
  echo "export GAZEBO_PLUGIN_PATH=\"$PLUG_PATH:\$GAZEBO_PLUGIN_PATH\"" >> ~/.bashrc
fi

echo "[INFO] Instalación completa. Recarga tu shell o ejecuta 'source ~/.bashrc' para aplicar los cambios."

