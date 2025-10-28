#!/bin/bash

# Script para verificar que todos los cambios necesarios estén presentes
# Guarda como: verify_custom_radius_changes.sh
# Ejecuta con: bash verify_custom_radius_changes.sh

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

SPLISHSPLASH_DIR="$HOME/SPlisHSPlasH"
CHANGES_OK=true

echo -e "${GREEN}=== Verificando cambios para radios personalizados ===${NC}\n"

# Verificar GazeboSceneLoader.h
echo -e "${YELLOW}1. Verificando GazeboSceneLoader.h...${NC}"
HEADER_FILE="$SPLISHSPLASH_DIR/Simulators/Common/GazeboSceneLoader.h"

if grep -q "std::map<std::string, Real> customBoundaryRadii" "$HEADER_FILE"; then
    echo -e "${GREEN}   ✓ Campo customBoundaryRadii encontrado en Scene${NC}"
else
    echo -e "${RED}   ✗ Campo customBoundaryRadii NO encontrado en Scene${NC}"
    CHANGES_OK=false
fi

if grep -q "bool useCustomRadius" "$HEADER_FILE"; then
    echo -e "${GREEN}   ✓ Campo useCustomRadius encontrado en GazeboBoundaryData${NC}"
else
    echo -e "${RED}   ✗ Campo useCustomRadius NO encontrado en GazeboBoundaryData${NC}"
    CHANGES_OK=false
fi

if grep -q "#include <map>" "$HEADER_FILE"; then
    echo -e "${GREEN}   ✓ Include <map> encontrado${NC}"
else
    echo -e "${RED}   ✗ Include <map> NO encontrado${NC}"
    CHANGES_OK=false
fi

# Verificar GazeboSceneLoader.cpp
echo -e "\n${YELLOW}2. Verificando GazeboSceneLoader.cpp...${NC}"
CPP_FILE="$SPLISHSPLASH_DIR/Simulators/Common/GazeboSceneLoader.cpp"

if grep -q "customBoundaryRadius" "$CPP_FILE"; then
    echo -e "${GREEN}   ✓ Código para leer customBoundaryRadius encontrado${NC}"
else
    echo -e "${RED}   ✗ Código para leer customBoundaryRadius NO encontrado${NC}"
    CHANGES_OK=false
fi

if grep -q "scene.customBoundaryRadii\[modelName\]" "$CPP_FILE"; then
    echo -e "${GREEN}   ✓ Asignación de radio personalizado encontrada${NC}"
else
    echo -e "${RED}   ✗ Asignación de radio personalizado NO encontrada${NC}"
    CHANGES_OK=false
fi

# Verificar GazeboBoundarySimulator.cpp
echo -e "\n${YELLOW}3. Verificando GazeboBoundarySimulator.cpp...${NC}"
BOUNDARY_FILE="$SPLISHSPLASH_DIR/Simulators/Common/GazeboBoundarySimulator.cpp"

if grep -q "boundaryRadius = scene.boundaryModels\[i\]->boundaryParticleRadius" "$BOUNDARY_FILE"; then
    echo -e "${GREEN}   ✓ Uso de radio específico por boundary encontrado${NC}"
else
    echo -e "${RED}   ✗ Uso de radio específico por boundary NO encontrado${NC}"
    CHANGES_OK=false
fi

# Verificar archivo SDF
echo -e "\n${YELLOW}4. Verificando pool_missile.sdf...${NC}"
SDF_FILE="$HOME/gazebo/pool_missile.sdf"

if [ -f "$SDF_FILE" ]; then
    if grep -q "<customBoundaryRadius>" "$SDF_FILE"; then
        echo -e "${GREEN}   ✓ Elemento customBoundaryRadius encontrado en SDF${NC}"
        
        if grep -q "<modelName>drone_extended</modelName>" "$SDF_FILE"; then
            echo -e "${GREEN}   ✓ Radio personalizado para drone_extended configurado${NC}"
        else
            echo -e "${YELLOW}   ⚠ Radio personalizado para drone_extended NO configurado${NC}"
        fi
        
        if grep -q "<modelName>biorob_pool</modelName>" "$SDF_FILE"; then
            echo -e "${GREEN}   ✓ Radio personalizado para biorob_pool configurado${NC}"
        else
            echo -e "${YELLOW}   ⚠ Radio personalizado para biorob_pool NO configurado${NC}"
        fi
    else
        echo -e "${YELLOW}   ⚠ Elemento customBoundaryRadius NO encontrado en SDF${NC}"
    fi
else
    echo -e "${YELLOW}   ⚠ Archivo pool_missile.sdf no encontrado en $SDF_FILE${NC}"
fi

# Resumen
echo -e "\n${GREEN}=== Resumen ===${NC}"
if [ "$CHANGES_OK" = true ]; then
    echo -e "${GREEN}✓ Todos los cambios necesarios están presentes${NC}"
    echo -e "${GREEN}Puedes proceder con la compilación:${NC}"
    echo -e "  cd $SPLISHSPLASH_DIR/build"
    echo -e "  make -j\$(nproc)"
    echo -e "  sudo make install"
else
    echo -e "${RED}✗ Faltan algunos cambios${NC}"
    echo -e "${YELLOW}Revisa los archivos mencionados arriba${NC}"
fi
