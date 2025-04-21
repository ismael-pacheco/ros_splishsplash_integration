#!/bin/bash
# update_packages.sh - Actualiza install_base.sh con los paquetes actualmente instalados

set -e

REPO_ROOT="$(dirname $(dirname $(realpath $0)))"

# 1. Crear backup de paquetes
mkdir -p "$REPO_ROOT/backups"
CURRENT_PKGS="apt-packages_$(date +%Y%m%d).txt"
apt-mark showmanual > "$REPO_ROOT/backups/$CURRENT_PKGS"

# 2. Generar lista de paquetes correctamente formateada
TOTAL_PKGS=$(apt-mark showmanual | wc -l)
CURRENT_LINE=0
PKG_LIST=$(apt-mark showmanual | while read pkg; do
  CURRENT_LINE=$((CURRENT_LINE+1))
  if [ $CURRENT_LINE -lt $TOTAL_PKGS ]; then
    echo "  $pkg \\"
  else
    echo "  $pkg"
  fi
done)

# 3. Actualizar install_base.sh
TEMP_FILE=$(mktemp)
awk -v pkgs="$PKG_LIST" '
  BEGIN {in_pkg_block = 0; first_line=1}
  /^# 2\. Update system and install all required dependencies/ {
    print
    print "sudo apt update && sudo apt install -y \\"
    printf "%s", pkgs
    in_pkg_block = 1
    next
  }
  in_pkg_block && /^[^ ]/ {
    in_pkg_block = 0
  }
  !in_pkg_block {print}
' "$REPO_ROOT/scripts/install_base.sh" > "$TEMP_FILE" && mv "$TEMP_FILE" "$REPO_ROOT/scripts/install_base.sh"

# 4. Actualizar git
cd "$REPO_ROOT"
git add "scripts/install_base.sh"
git commit -m "Actualización automática de paquetes $(date +%Y-%m-%d)"
echo "install_base.sh actualizado correctamente"
