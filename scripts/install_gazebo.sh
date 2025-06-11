#!/bin/bash
# install_gazebo_ros_noetic.sh - Installs HBP Gazebo from source with ROS Noetic integration

set -e

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}[INFO] Instalando Gazebo HBP Neurorobotics con ROS Noetic${NC}"

# Detect script directory to resolve relative paths
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# Verificar que ROS Noetic esté instalado
if ! command -v roscore &> /dev/null; then
    echo -e "${RED}[ERROR] ROS Noetic no está instalado o no está en PATH${NC}"
    exit 1
fi

echo -e "${YELLOW}[WARNING] Este script reemplazará la instalación actual de Gazebo${NC}"
echo -e "${YELLOW}Se recomienda hacer backup de configuraciones importantes${NC}"
read -p "¿Continuar? (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Instalación cancelada"
    exit 1
fi

# Remover instalación anterior de Gazebo si existe
echo -e "${GREEN}[1/6] Limpiando instalaciones anteriores de Gazebo...${NC}"
sudo apt remove -y gazebo11* libgazebo11* || true
sudo apt autoremove -y

# Instalar dependencias
echo -e "${GREEN}[2/6] Instalando dependencias...${NC}"
sudo apt update && sudo apt install -y \
    build-essential \
    cmake \
    git \
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
    libgraphviz-dev \
    libboost-all-dev \
    libxml2-dev \
    pkg-config \
    libqt5core5a \
    libqt5gui5 \
    libqt5opengl5-dev \
    libqt5widgets5 \
    qtbase5-dev \
    libtinyxml-dev \
    libtar-dev \
    libtbb-dev \
    libogre-1.9-dev \
    libcurl4-openssl-dev \
    cppcheck \
    libbullet-dev \
    libsdformat9-dev \
    libignition-cmake2-dev

# Clonar repositorio HBP Gazebo
echo -e "${GREEN}[3/6] Clonando repositorio Gazebo HBP...${NC}"
cd ~
if [ ! -d "gazebo" ]; then
    git clone https://bitbucket.org/hbpneurorobotics/gazebo.git
else
    echo "Directorio gazebo ya existe, actualizando..."
    cd ~/gazebo
    git checkout development
    git pull origin development
    cd ~
fi

cd ~/gazebo
git checkout development

# Preparar CMakeLists.txt para C++14
echo -e "${GREEN}[4/6] Configurando CMakeLists.txt...${NC}"
# Backup original
cp CMakeLists.txt CMakeLists.txt.backup

# Inyectar requerimiento C++14
if ! grep -q "CMAKE_CXX_STANDARD 14" CMakeLists.txt; then
    echo "[INFO] Agregando estándar C++14 a CMakeLists.txt..."
    sed -i '1iset(CMAKE_CXX_STANDARD 14)\nset(CMAKE_CXX_STANDARD_REQUIRED ON)' CMakeLists.txt
fi

# Construir Gazebo
echo -e "${GREEN}[5/6] Construyendo Gazebo...${NC}"
if [ ! -d "build" ]; then
    mkdir build
fi
cd build

# Limpiar build anterior si existe
rm -rf *

# Configurar con CMAKE para instalación en $HOME/.local (compatible con SPlisHSPlasH)
cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local \
      -DCMAKE_BUILD_TYPE=Release \
      -DUSE_HOST_CFLAGS=OFF \
      -DCMAKE_PREFIX_PATH="$HOME/.local;/usr/local" \
      ..

# Compilar con número apropiado de jobs
NPROC=4
echo "[INFO] Compilando con $NPROC procesos paralelos..."
make -j$NPROC 2> errors.log

if [ $? -ne 0 ]; then
    echo -e "${RED}[ERROR] Falló la compilación. Revisando errors.log...${NC}"
    tail -20 errors.log
    exit 1
fi

# Instalar Gazebo
echo -e "${GREEN}[6/6] Instalando Gazebo...${NC}"
make install
# Actualizar ldconfig para que encuentre las librerías en ~/.local
echo "$HOME/.local/lib" | sudo tee /etc/ld.so.conf.d/local-user-libs.conf
sudo ldconfig

# Configurar variables de entorno
echo -e "${GREEN}[INFO] Configurando variables de entorno...${NC}"

# Crear archivo de configuración para ROS
GAZEBO_SETUP_FILE="$HOME/.gazebo_hbp_setup.sh"
cat > "$GAZEBO_SETUP_FILE" << 'EOF'
#!/bin/bash
# Configuración para Gazebo HBP Neurorobotics (compatible con SPlisHSPlasH)

# Solo cargar si no está ya cargado
if [ -z "$GAZEBO_HBP_LOADED" ]; then
    # Agregar ~/.local a PATH si no está
    if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
        export PATH="$HOME/.local/bin:$PATH"
    fi

    # Variables de entorno de Gazebo
    export GAZEBO_MODEL_PATH="$HOME/.local/share/gazebo-11/models:$GAZEBO_MODEL_PATH"
    export GAZEBO_RESOURCE_PATH="$HOME/.local/share/gazebo-11:$GAZEBO_RESOURCE_PATH" 
    export GAZEBO_PLUGIN_PATH="$HOME/.local/lib/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH"
    export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"
    export PKG_CONFIG_PATH="$HOME/.local/lib/pkgconfig:$PKG_CONFIG_PATH"

    # Configuración específica para HBP
    export HBP_GAZEBO_MODELS_PATH="$HOME/.local/share/gazebo-11/models"
    export HBP_GAZEBO_WORLDS_PATH="$HOME/.local/share/gazebo-11/worlds"

    # Compatibilidad con SPlisHSPlasH
    export CMAKE_PREFIX_PATH="$HOME/.local:$CMAKE_PREFIX_PATH"

    # Para que ROS encuentre Gazebo en ~/.local
    export GAZEBO_MASTER_URI="http://localhost:11345"
    export GAZEBO_MODEL_DATABASE_URI="http://gazebosim.org/models"
    
    # Marcar como cargado
    export GAZEBO_HBP_LOADED=1
fi
EOF

# Agregar al bashrc si no está ya
if ! grep -q "gazebo_hbp_setup.sh" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# HBP Gazebo Configuration (auto-loaded)" >> ~/.bashrc
    echo "source $GAZEBO_SETUP_FILE" >> ~/.bashrc
    echo "[INFO] Configuración de Gazebo HBP agregada a .bashrc (se carga automáticamente)"
fi

# Copiar archivos de test si existen
if [ -d "$REPO_ROOT/test files" ]; then
    echo -e "${GREEN}[INFO] Copiando archivos de test...${NC}"
    cp -r "$REPO_ROOT/test files/"* "$HOME/gazebo/" 2>/dev/null || true
fi

# Crear script de verificación
VERIFY_SCRIPT="$HOME/verify_gazebo_hbp.sh"
cat > "$VERIFY_SCRIPT" << 'EOF'
#!/bin/bash
echo "=== Verificación de instalación Gazebo HBP (compatible con SPlisHSPlasH) ==="
echo "Versión de Gazebo:"
$HOME/.local/bin/gazebo --version

echo -e "\nUbicación de ejecutables:"
which gazebo || echo "No encontrado en PATH - usando: $HOME/.local/bin/gazebo"
ls -la $HOME/.local/bin/gazebo* 2>/dev/null || echo "Ejecutables no encontrados en ~/.local/bin"

echo -e "\nPlugins disponibles:"
ls $HOME/.local/lib/gazebo-11/plugins/ 2>/dev/null | head -10 || echo "Directorio de plugins no encontrado"

echo -e "\nModelos disponibles:"
ls $HOME/.local/share/gazebo-11/models/ 2>/dev/null | head -10 || echo "Directorio de modelos no encontrado"

echo -e "\nLibrerías instaladas:"
ls $HOME/.local/lib/libgazebo* 2>/dev/null | head -5 || echo "Librerías no encontradas"

echo -e "\nVariables de entorno importantes:"
echo "PATH: $PATH"
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
echo "GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"

echo -e "\nVerificación de compatibilidad con SPlisHSPlasH:"
if [ -f "$HOME/.local/bin/DynamicBoundarySimulator" ]; then
    echo "✓ SPlisHSPlasH encontrado en ~/.local/bin"
else
    echo "⚠ SPlisHSPlasH no encontrado (instálalo después)"
fi

echo -e "\nPrueba de lanzamiento (dry-run):"
echo "Comando para probar: $HOME/.local/bin/gazebo --verbose"
EOF

chmod +x "$VERIFY_SCRIPT"

echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}✓ Gazebo HBP instalado exitosamente${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo -e "${YELLOW}PASOS SIGUIENTES:${NC}"
echo "1. Reinicia tu terminal o ejecuta: source ~/.bashrc"
echo "2. Verifica la instalación: bash ~/verify_gazebo_hbp.sh"
echo "3. Prueba con: gazebo --version"
echo "4. Para usar con ROS: roslaunch gazebo_ros empty_world.launch"
echo ""
echo -e "${YELLOW}ARCHIVOS IMPORTANTES:${NC}"
echo "- Script de verificación: ~/verify_gazebo_hbp.sh"
echo "- Configuración: ~/.gazebo_hbp_setup.sh"
echo "- Log de errores: ~/gazebo/build/errors.log"
echo ""
echo -e "${GREEN}Instalación completada en $HOME/.local${NC}"
echo ""
echo -e "${YELLOW}COMPATIBILIDAD CON SPLISHSPLASH:${NC}"
echo "- Ambos proyectos usan $HOME/.local como prefijo"
echo "- Las variables CMAKE_PREFIX_PATH están configuradas"
echo "- Instala SPlisHSPlasH después de Gazebo para mejor compatibilidad"
