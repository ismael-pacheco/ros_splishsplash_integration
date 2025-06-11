\#!/bin/bash
set -e

# install\_px4.sh - Script para instalar y compilar PX4 SITL sin Gazebo

# Autor: Ismael Pacheco Villegas

# Fecha: \$(date +"%Y-%m-%d")

echo "\[INFO] Iniciando instalación de PX4 SITL..."

# 1) Dependencias del sistema

echo "\[INFO] Instalando dependencias del sistema..."
sudo apt update
sudo apt install -y&#x20;
git&#x20;
wget&#x20;
zip&#x20;
cmake&#x20;
build-essential&#x20;
genromfs&#x20;
python3&#x20;
python3-pip&#x20;
python3-venv&#x20;
python3-colour&#x20;
libopencv-dev&#x20;
ccache

# 2) Clonar o actualizar PX4-Autopilot

PX4\_DIR="\$HOME/PX4-Autopilot"
if \[ -d "\$PX4\_DIR" ]; then
echo "\[INFO] PX4-Autopilot ya existe en \$PX4\_DIR. Actualizando..."
cd "\$PX4\_DIR" && git pull
else
echo "\[INFO] Clonando PX4-Autopilot en \$PX4\_DIR..."
git clone [https://github.com/PX4/PX4-Autopilot.git](https://github.com/PX4/PX4-Autopilot.git) "\$PX4\_DIR"
cd "\$PX4\_DIR"
fi

# 3) Inicializar submódulos

echo "\[INFO] Inicializando submódulos de Git..."
git submodule update --init --recursive

# 4) Crear entorno virtual de Python

echo "\[INFO] Configurando entorno virtual de Python..."
PY\_ENV="\$PX4\_DIR/venv-px4"
if \[ ! -d "\$PY\_ENV" ]; then
python3 -m venv "\$PY\_ENV"
fi
source "\$PY\_ENV/bin/activate"

# 5) Instalar requerimientos Python

echo "\[INFO] Instalando dependencias Python..."
pip install --upgrade pip
echo "- Instalando herramientas SITL"
pip install --user -r Tools/setup/requirements.txt
pip install --user future pymavlink MAVProxy

# 6) Compilar SITL para PX4 (sin Gazebo)

echo "\[INFO] Compilando PX4 SITL..."
cd "\$PX4\_DIR"
make px4\_sitl\_default gazebo\_nogui

# 7) Opcional: compilar con interfaz mínima

# make px4\_sitl\_default jmavsim

# 8) Mensaje final

echo "\[INFO] PX4 SITL instalado y compilado correctamente."
echo "Para iniciar SITL sin Gazebo, ejecute:"
echo "  cd \$PX4\_DIR && source \$PY\_ENV/bin/activate && make px4\_sitl\_default gazebo\_nogui"

deactivate

