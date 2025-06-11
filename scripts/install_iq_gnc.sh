#!/bin/bash
# install_iq_gnc.sh - Configura el paquete IQ_GNC en un workspace Catkin

set -e
echo "[INFO] Configurando workspace Catkin para IQ_GNC..."
# Definir workspace
WS=~/catkin_ws
SRC=\$WS/src

# Crear workspace si no existe
if [ ! -d \"\$WS\" ]; then
  mkdir -p \"\$SRC\"
fi

cd \"\$SRC\"

# Clonar el repositorio iq_gnc
if [ -d \"iq_gnc\" ]; then
  echo "[INFO] iq_gnc ya existe, actualizando..."
  cd iq_gnc && git pull
else
  git clone https://github.com/Intelligent-Quads/iq_gnc.git
  cd iq_gnc
fi

# Instalar dependencias
cd \"$WS\"
rosdep update
rosdep install --from-paths src -i -y

# Compilar
catkin_make

# Fuente de entorno
echo "[INFO] Añadiendo source al .bashrc..."
if ! grep -Fxq "source \$WS/devel/setup.bash" ~/.bashrc; then
  echo "source \$WS/devel/setup.bash" >> ~/.bashrc
fi
source ~/.bashrc

echo "[INFO] IQ_GNC instalado y compilado en \$WS."

