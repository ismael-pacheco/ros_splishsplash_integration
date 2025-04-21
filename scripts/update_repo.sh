#!/bin/bash
# update_repo.sh - Actualiza todo el entorno y el repositorio

set -e

echo "Actualizando sistema y repositorio..."

# 1. Actualizar paquetes del sistema
sudo apt update && sudo apt upgrade -y

# 2. Ejecutar el actualizador de paquetes
./scripts/update_packages.sh

# 3. Sincronizar con el repositorio remoto (si existe)
if git remote -v | grep -q "origin"; then
    git pull origin main
    git push origin main
fi

echo "Sistema y repositorio actualizados"
