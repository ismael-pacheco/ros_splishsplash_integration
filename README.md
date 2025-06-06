# ROS + SPlisHSPlasH Integration Suite

<p align="center">
  <img src="https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg" alt="ROS Logo" height="80"/>
  <img src="https://upload.wikimedia.org/wikipedia/commons/5/5e/Gazebo_logo_without_text.svg" alt="Gazebo Logo" height="80"/>
  <img src="https://raw.githubusercontent.com/InteractiveComputerGraphics/SPlisHSPlasH/master/doc/images/logo.jpg" alt="SPlisHSPlasH Logo" height="80"/>
</p>

> Paquete de integración para simulaciones de fluidos con **SPlisHSPlasH** en entornos **ROS/Gazebo**.

---

## 📋 Tabla de Contenidos
- [Requisitos Previos](#-requisitos-previos)
- [Instalación](#-instalación)
- [Mantenimiento](#-mantenimiento)
- [Estructura del Proyecto](#-estructura-del-proyecto)
- [Uso Básico](#-uso-básico)
- [Soporte Técnico](#-soporte-técnico)
- [Licencia](#-licencia)

---

## 📦 Requisitos Previos

### Hardware
- **Sistema Operativo:** Ubuntu 20.04 LTS  
- **RAM:** 8GB mínimo (16GB recomendado)  
- **Almacenamiento:** 20GB espacio libre  
- **GPU:** NVIDIA con soporte CUDA (recomendado)

### Software
- Git `2.25+`  
- CMake `3.16+`  
- Python `3.8`

---

## 🚀 Instalación

### Clonación del Repositorio

```bash
git clone --recursive https://github.com/ismael-pacheco/ros_splishsplash_integration.git
cd ros_splishsplash_integration
```

---

### Instalación Automática (Modo Completo)

```bash
# Ejecutar todos los scripts en orden (≈60-90 mins)
./install_all.sh
```

---

### Instalación Manual (Paso a Paso)


#### Dependencias Base:
```bash
./scripts/install_base.sh
```

#### Entorno ROS:
```bash
./scripts/install_ros.sh
```

#### Componentes de Simulación:
```bash
./scripts/install_gazebo.sh
./scripts/install_splishsplash.sh
```

#### Configuración Final:
```bash
./scripts/setup_environment.sh
source ~/.bashrc
```

---

## 🔄 Mantenimiento

### Actualización del Sistema:
```bash
./scripts/update_repo.sh
```

### Limpieza:
```bash
./scripts/clean_builds.sh
```

---

## 📂 Estructura del Proyecto

```
ros_splishsplash_integration/
├── scripts/               # Scripts de instalación
│   ├── install_*.sh       # Scripts individuales
│   └── update_repo.sh     # Actualizador
├── backups/               # Copias de seguridad
│   └── apt-packages_*.txt # Historial de paquetes
├── config/                # Archivos de configuración
├── docs/                  # Documentación técnica
└── logs/                  # Registros de instalación
```

---

## 🖥️ Uso Básico

```bash
cd ~/gazebo
gazebo pool_with_falling_box.sdf -g libFluidVisPlugin.so
```

---

## ❓ Soporte Técnico

### Diagnóstico de Problemas

Consultar logs:
```bash
tail -n 50 logs/install_*.log
```

Verificar dependencias:
```bash
rosdep check --from-paths src --ignore-src
```

### Canal de Soporte
- Reportar *issues* vía GitHub

---

## 📜 Licencia

Este proyecto está bajo la licencia **MIT**.

---

### ⚠️ Notas Importantes
- 💡 Ejecutar `source ~/.bashrc` tras la instalación completa

---

<div align="center">
  <sub>Creado con ❤️ por <a href="https://github.com/ismael-pacheco">Ismael Pacheco</a></sub>
</div>

