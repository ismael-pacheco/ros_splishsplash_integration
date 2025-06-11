#!/bin/bash
# manage_bashrc_config.sh - Gestionar configuración automática en .bashrc

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

show_help() {
    echo "Gestión de configuración Gazebo HBP en .bashrc"
    echo ""
    echo "Uso: $0 [opción]"
    echo ""
    echo "Opciones:"
    echo "  enable    - Habilitar carga automática en .bashrc"
    echo "  disable   - Deshabilitar carga automática en .bashrc"
    echo "  status    - Mostrar estado actual"
    echo "  reload    - Recargar configuración en terminal actual"
    echo "  test      - Probar configuración actual"
    echo "  help      - Mostrar esta ayuda"
}

check_status() {
    echo -e "${YELLOW}=== Estado de configuración .bashrc ===${NC}"
    
    if grep -q "gazebo_hbp_setup.sh" ~/.bashrc; then
        echo -e "${GREEN}✓ Configuración básica Gazebo HBP: HABILITADA${NC}"
    else
        echo -e "${RED}✗ Configuración básica Gazebo HBP: DESHABILITADA${NC}"
    fi
    
    if grep -q "ros_gazebo_hbp_setup.sh" ~/.bashrc; then
        echo -e "${GREEN}✓ Configuración ROS-Gazebo HBP: HABILITADA${NC}"
    else
        echo -e "${RED}✗ Configuración ROS-Gazebo HBP: DESHABILITADA${NC}"
    fi
    
    echo ""
    echo "Variables de entorno actuales:"
    echo "GAZEBO_HBP_LOADED: ${GAZEBO_HBP_LOADED:-NO}"
    echo "ROS_GAZEBO_HBP_LOADED: ${ROS_GAZEBO_HBP_LOADED:-NO}"
    echo "ROS_DISTRO: ${ROS_DISTRO:-NO}"
}

enable_config() {
    echo -e "${GREEN}Habilitando configuración automática...${NC}"
    
    # Verificar que los archivos de configuración existan
    if [ ! -f ~/.gazebo_hbp_setup.sh ]; then
        echo -e "${RED}Error: ~/.gazebo_hbp_setup.sh no existe${NC}"
        echo "Ejecuta primero install_gazebo_ros_noetic.sh"
        return 1
    fi
    
    # Habilitar configuración básica Gazebo
    if ! grep -q "gazebo_hbp_setup.sh" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# HBP Gazebo Configuration (auto-loaded)" >> ~/.bashrc
        echo "source ~/.gazebo_hbp_setup.sh" >> ~/.bashrc
        echo -e "${GREEN}✓ Configuración básica Gazebo HBP agregada${NC}"
    else
        echo "✓ Configuración básica Gazebo HBP ya está habilitada"
    fi
    
    # Habilitar configuración ROS-Gazebo si existe
    if [ -f ~/.ros_gazebo_hbp_setup.sh ]; then
        if ! grep -q "ros_gazebo_hbp_setup.sh" ~/.bashrc; then
            echo "" >> ~/.bashrc
            echo "# ROS-Gazebo HBP Configuration (auto-loaded)" >> ~/.bashrc
            echo "source ~/.ros_gazebo_hbp_setup.sh" >> ~/.bashrc
            echo -e "${GREEN}✓ Configuración ROS-Gazebo HBP agregada${NC}"
        else
            echo "✓ Configuración ROS-Gazebo HBP ya está habilitada"
        fi
    else
        echo -e "${YELLOW}⚠ ~/.ros_gazebo_hbp_setup.sh no existe${NC}"
        echo "Ejecuta configure_ros_gazebo_local.sh si necesitas integración ROS"
    fi
    
    echo ""
    echo -e "${GREEN}Configuración habilitada. Reinicia tu terminal o ejecuta:${NC}"
    echo "source ~/.bashrc"
}

disable_config() {
    echo -e "${YELLOW}Deshabilitando configuración automática...${NC}"
    
    # Crear backup
    cp ~/.bashrc ~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)
    
    # Remover configuración Gazebo HBP
    sed -i '/# HBP Gazebo Configuration/,+1d' ~/.bashrc
    sed -i '/source.*gazebo_hbp_setup.sh/d' ~/.bashrc
    
    # Remover configuración ROS-Gazebo HBP
    sed -i '/# ROS-Gazebo HBP Configuration/,+1d' ~/.bashrc
    sed -i '/source.*ros_gazebo_hbp_setup.sh/d' ~/.bashrc
    
    echo -e "${GREEN}✓ Configuración automática deshabilitada${NC}"
    echo "Backup creado: ~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)"
    echo ""
    echo "Para aplicar cambios, reinicia tu terminal o ejecuta:"
    echo "source ~/.bashrc"
}

reload_config() {
    echo -e "${GREEN}Recargando configuración...${NC}"
    
    # Limpiar variables para forzar recarga
    unset GAZEBO_HBP_LOADED ROS_GAZEBO_HBP_LOADED
    
    # Recargar .bashrc
    source ~/.bashrc
    
    echo -e "${GREEN}✓ Configuración recargada${NC}"
}

test_config() {
    echo -e "${YELLOW}=== Prueba de configuración ===${NC}"
    
    echo "1. Verificando Gazebo HBP..."
    if command -v gazebo &> /dev/null; then
        echo -e "${GREEN}✓ Comando 'gazebo' disponible${NC}"
        gazebo --version | head -1
    else
        echo -e "${RED}✗ Comando 'gazebo' no disponible${NC}"
    fi
    
    echo ""
    echo "2. Verificando ROS..."
    if command -v roscore &> /dev/null; then
        echo -e "${GREEN}✓ ROS disponible (${ROS_DISTRO:-desconocido})${NC}"
    else
        echo -e "${RED}✗ ROS no disponible${NC}"
    fi
    
    echo ""
    echo "3. Verificando funciones personalizadas..."
    if type launch_gazebo_ros &> /dev/null; then
        echo -e "${GREEN}✓ Función 'launch_gazebo_ros' disponible${NC}"
    else
        echo -e "${YELLOW}⚠ Función 'launch_gazebo_ros' no disponible${NC}"
    fi
    
    if type check_gazebo_ros &> /dev/null; then
        echo -e "${GREEN}✓ Función 'check_gazebo_ros' disponible${NC}"
    else
        echo -e "${YELLOW}⚠ Función 'check_gazebo_ros' no disponible${NC}"
    fi
    
    echo ""
    echo "4. Variables de entorno importantes:"
    echo "   GAZEBO_MODEL_PATH: ${GAZEBO_MODEL_PATH:-(no configurado)}"
    echo "   GAZEBO_PLUGIN_PATH: ${GAZEBO_PLUGIN_PATH:-(no configurado)}"
    echo "   PATH incluye ~/.local/bin: $(echo $PATH | grep -q "$HOME/.local/bin" && echo "Sí" || echo "No")"
}

# Procesamiento de argumentos
case "${1:-help}" in
    enable)
        enable_config
        ;;
    disable)
        disable_config
        ;;
    status)
        check_status
        ;;
    reload)
        reload_config
        ;;
    test)
        test_config
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        echo -e "${RED}Opción no reconocida: $1${NC}"
        echo ""
        show_help
        exit 1
        ;;
esac
