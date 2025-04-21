# ROS + SPlisHSPlasH Integration Suite

![ROS Logo](https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg) 
![Gazebo Logo](https://upload.wikimedia.org/wikipedia/commons/thumb/8/8e/Gazebo_logo.svg/1200px-Gazebo_logo.svg.png) 
![SPlisHSPlasH Logo](https://hbpneurorobotics.files.wordpress.com/2017/10/splishsplash_logo.png)

Paquete de integración para simulaciones de fluidos con SPlisHSPlasH en entornos ROS/Gazebo.

## 📋 Tabla de Contenidos
- [Requisitos](#-requisitos-previos)
- [Instalación](#-instalación)
- [Mantenimiento](#-mantenimiento)
- [Estructura](#-estructura-del-proyecto)
- [Uso Básico](#-uso-básico)
- [Soporte](#-soporte)
- [Licencia](#-licencia)

## 📦 Requisitos Previos

### Hardware
- **Sistema Operativo**: Ubuntu 20.04 LTS
- **RAM**: 8GB mínimo (16GB recomendado)
- **Almacenamiento**: 20GB espacio libre
- **GPU**: NVIDIA con soporte CUDA (recomendado)

### Software
- Git 2.25+
- CMake 3.16+
- Python 3.8

## 🚀 Instalación

### Clonación del Repositorio
```bash
git clone --recursive https://github.com/ismael-pacheco/ros_splishsplash_integration.git
cd ros_splishsplash_integration

Instalación Automática (Modo Completo)
bash

# Ejecutar todos los scripts en orden (≈60-90 mins)
./install_all.sh  # Si dispones de este script consolidado

# O ejecutar manualmente:
for script in scripts/install_{base,ros,ruby,sdformat,ignition_math,gazebo,splishsplash,environment}.sh; do
    echo "🔧 Ejecutando $script..."
    chmod +x $script && ./$script
done

Instalación Manual (Paso a Paso)

    Dependencias Base:
    bash

./scripts/install_base.sh

Entorno ROS:
bash

./scripts/install_ros.sh

Componentes de Simulación:
bash

./scripts/install_gazebo.sh
./scripts/install_splishsplash.sh

Configuración Final:
bash

    ./scripts/setup_environment.sh
    source ~/.bashrc

🔄 Mantenimiento
Actualización del Sistema
bash

./scripts/update_repo.sh  # Actualiza repositorio y dependencias

Limpieza
bash

./scripts/clean_builds.sh  # Elimina archivos temporales de compilación

📂 Estructura del Proyecto

ros_splishsplash_integration/
├── scripts/               # Scripts de instalación
│   ├── install_*.sh       # Scripts individuales
│   └── update_repo.sh     # Actualizador
├── backups/               # Copias de seguridad
│   └── apt-packages_*.txt # Historial de paquetes
├── config/                # Archivos de configuración
├── docs/                  # Documentación técnica
└── logs/                  # Registros de instalación

🖥️ Uso Básico
Ejecutar Simulación de Prueba
bash

roslaunch splishsplash_demo fluid_simulation.launch

Verificar Instalación
bash

./scripts/verify_installation.sh  # Script de verificación

❓ Soporte Técnico
Diagnóstico de Problemas

    Consultar logs en logs/:
    bash

tail -n 50 logs/install_*.log

Verificar dependencias:
bash

    rosdep check --from-paths src --ignore-src

Canal de Soporte

    Reportar Issues

    Foro de discusión: #ros-splishsplash en Discord

📜 Licencia

Este proyecto está bajo licencia MIT.

    Notas Importantes:
    ⚠️ Todos los scripts requieren conexión a Internet estable
    ⏱️ Tiempos estimados en hardware medio (i7, 16GB RAM, SSD)
    💡 Ejecutar source ~/.bashrc tras la instalación completa

<div align="center"> <sub>Creado con ❤️ por <a href="https://github.com/ismael-pacheco">Ismael Pacheco</a></sub> </div> ```
