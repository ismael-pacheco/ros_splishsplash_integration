#!/bin/bash
# configure_ros_gazebo_local.sh - Configura ROS Noetic para usar Gazebo HBP desde ~/.local

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Mostrar ayuda si se pide
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    echo "Usage: configure_ros_gazebo_local.sh"
    echo
    echo "Este script instala y configura:"
    echo "  - ~/.ros_gazebo_hbp_setup.sh       # carga automática en .bashrc"
    echo "  - ~/.local/bin/roslaunch_gazebo_hbp # wrapper de roslaunch"
    echo "  - ~/.local/share/gazebo_ros_hbp/empty_world_hbp.launch"
    exit 0
fi

echo -e "${GREEN}Configurando ROS Noetic para usar Gazebo HBP desde ~/.local${NC}"

# Verificar que ROS esté instalado
if ! command -v roscore &> /dev/null; then
    echo -e "${RED}[ERROR] ROS Noetic no está instalado${NC}"
    exit 1
fi

# Verificar que Gazebo HBP esté instalado
if [ ! -f "$HOME/.local/bin/gazebo" ]; then
    echo -e "${RED}[ERROR] Gazebo HBP no está instalado en ~/.local${NC}"
    echo "Ejecuta primero el script install_gazebo_ros_noetic.sh"
    exit 1
fi

# Variables de ubicación
ROS_GAZEBO_SETUP="$HOME/.ros_gazebo_hbp_setup.sh"
ROSLAUNCH_WRAPPER="$HOME/.local/bin/roslaunch_gazebo_hbp"
CUSTOM_LAUNCH_DIR="$HOME/.local/share/gazebo_ros_hbp"

# -------------------------------
# 1) Crear archivo de configuración ROS-Gazebo
# -------------------------------
cat > "$ROS_GAZEBO_SETUP" << 'EOF'
#!/bin/bash
# Configuración ROS-Gazebo HBP (carga automática en .bashrc)

# Evita duplicar carga
if [ -z "$ROS_GAZEBO_HBP_LOADED" ]; then
    # Si ROS no está cargado, hazlo
    if [ -z "$ROS_DISTRO" ]; then
        source /opt/ros/noetic/setup.bash
    fi

    # Carga setup de Gazebo HBP si existe
    if [ -f ~/.gazebo_hbp_setup.sh ]; then
        source ~/.gazebo_hbp_setup.sh
    fi

    # URI del master de Gazebo HBP
    export GAZEBO_MASTER_URI="http://localhost:11345"

    # Fuerza uso de binarios locales
    alias gazebo="$HOME/.local/bin/gazebo"
    alias gzserver="$HOME/.local/bin/gzserver"
    alias gzclient="$HOME/.local/bin/gzclient"

    # Función: lanzar Gazebo bajo ROS
    function launch_gazebo_ros() {
        echo "Lanzando Gazebo HBP con ROS..."
        roslaunch gazebo_ros empty_world.launch \
            gui:=true \
            paused:=false \
            use_sim_time:=true \
            verbose:=false \
            debug:=false \
            respawn_gazebo:=false \
            "$@"
    }

    # Función: chequeo de configuración
    function check_gazebo_ros() {
        echo "=== Estado de ROS-Gazebo HBP ==="
        echo "ROS_DISTRO: $ROS_DISTRO"
        echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
        echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
        echo "GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"
        echo "PATH (gazebo): $(which gazebo 2>/dev/null || echo 'No encontrado en PATH')"
        echo
        echo "Gazebo HBP version:"
        $HOME/.local/bin/gazebo --version 2>/dev/null || echo "Gazebo HBP no encontrado"
        echo
        echo "ROS packages gazebo:"
        rospack list | grep gazebo 2>/dev/null || echo "No gazebo packages found"
        echo
        echo "Prueba: launch_gazebo_ros"
    }

    # Función: desactivar configuración en la sesión
    function disable_gazebo_hbp() {
        unalias gazebo gzserver gzclient 2>/dev/null || true
        unset -f launch_gazebo_ros check_gazebo_ros disable_gazebo_hbp
        echo "Configuración Gazebo HBP desactivada para esta sesión"
    }

    # Indicador de carga
    export ROS_GAZEBO_HBP_LOADED=1

    # Mensaje al usuario en terminal interactiva
    if [[ $- == *i* ]]; then
        echo "✅ ROS-Gazebo HBP configurado. Comandos: launch_gazebo_ros, check_gazebo_ros"
    fi
fi
EOF

chmod +x "$ROS_GAZEBO_SETUP"

# -------------------------------
# 2) Crear el wrapper roslaunch
# -------------------------------
mkdir -p "$(dirname "$ROSLAUNCH_WRAPPER")"
cat > "$ROSLAUNCH_WRAPPER" << 'EOF'
#!/bin/bash
# Wrapper para roslaunch con Gazebo HBP

# Carga configuración
source ~/.ros_gazebo_hbp_setup.sh

# Lanza roslaunch con todos los argumentos
exec roslaunch "$@"
EOF

chmod +x "$ROSLAUNCH_WRAPPER"

# -------------------------------
# 3) Crear launch file personalizado
# -------------------------------
mkdir -p "$CUSTOM_LAUNCH_DIR"
cat > "$CUSTOM_LAUNCH_DIR/empty_world_hbp.launch" << 'EOF'
<?xml version="1.0"?>
<launch>

  <!-- argumentos disponibles -->
  <arg name="paused"      default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui"         default="true"/>
  <arg name="headless"    default="false"/>
  <arg name="debug"       default="false"/>
  <arg name="verbose"     default="true"/>
  <arg name="world_name"  default="worlds/empty.world"/>

  <!-- Ajusta use_sim_time -->
  <param name="/use_sim_time" value="$(arg use_sim_time)"/>

  <!-- Parámetros de línea de comandos -->
  <arg unless="$(arg paused)"   name="command_arg1" value=""/>
  <arg     if="$(arg paused)"   name="command_arg1" value="-u"/>
  <arg unless="$(arg headless)" name="command_arg2" value=""/>
  <arg     if="$(arg headless)" name="command_arg2" value="-r"/>
  <arg unless="$(arg verbose)"  name="command_arg3" value=""/>
  <arg     if="$(arg verbose)"  name="command_arg3" value="--verbose"/>
  <arg unless="$(arg debug)"    name="script_type" value="gzserver"/>
  <arg     if="$(arg debug)"    name="script_type" value="debug"/>

  <!-- Lanzar servidor -->
  <group if="$(arg use_sim_time)">
    <param name="/use_sim_time" value="true" />
  </group>
  <node name="gazebo" pkg="gazebo_ros" type="$(arg script_type)" respawn="false" output="screen"
        args="$(arg command_arg1) $(arg command_arg2) $(arg command_arg3) $(arg world_name)"
        launch-prefix="env GAZEBO_COMMAND=$(env HOME)/.local/bin/gazebo"/>

  <!-- Lanzar cliente GUI -->
  <group if="$(arg gui)">
    <node name="gazebo_gui" pkg="gazebo_ros" type="gzclient" respawn="false" output="screen"
          launch-prefix="env GAZEBO_COMMAND=$(env HOME)/.local/bin/gzclient"/>
  </group>

</launch>
EOF

# -------------------------------
# 4) Añadir carga automática en ~/.bashrc
# -------------------------------
if ! grep -q "ros_gazebo_hbp_setup.sh" ~/.bashrc; then
    echo ""                                >> ~/.bashrc
    echo "# ROS-Gazebo HBP Configuration (auto-loaded)" >> ~/.bashrc
    echo "source $ROS_GAZEBO_SETUP"       >> ~/.bashrc
    echo "[INFO] Configuración ROS-Gazebo HBP agregada a .bashrc (se carga automáticamente)"
else
    echo "[INFO] Configuración ya existe en .bashrc"
fi

# -------------------------------
# 5) Mensajes finales de uso
# -------------------------------
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}✓ Configuración ROS-Gazebo HBP completada${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo -e "${YELLOW}CÓMO USAR (configuración automática):${NC}"
echo "1. ✅ La configuración se carga automáticamente en nuevas terminales"
echo "2. Para aplicar ahora: source ~/.bashrc"
echo ""
echo "3. Comandos disponibles en cualquier terminal:"
echo "   launch_gazebo_ros    - Lanza Gazebo con ROS"
echo "   check_gazebo_ros     - Verifica configuración"
echo "   gazebo               - Comando Gazebo HBP"
echo ""
echo "4. Para lanzar mundo vacío:"
echo "   launch_gazebo_ros"
echo ""
echo "5. Para usar launch files estándar:"
echo "   roslaunch gazebo_ros empty_world.launch"
echo ""
echo -e "${YELLOW}ARCHIVOS CREADOS:${NC}"
echo "- Configuración: $ROS_GAZEBO_SETUP"
echo "- Wrapper:       $ROSLAUNCH_WRAPPER"
echo "- Launch file:   $CUSTOM_LAUNCH_DIR/empty_world_hbp.launch"

