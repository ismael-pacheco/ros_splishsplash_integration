# ROS + SPlisHSPlasH Integration Suite

<p align="center">
  <img src="https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg" alt="ROS Logo" height="80"/>
  <img src="https://upload.wikimedia.org/wikipedia/commons/5/5e/Gazebo_logo_without_text.svg" alt="Gazebo Logo" height="80"/>
  <img src="https://raw.githubusercontent.com/InteractiveComputerGraphics/SPlisHSPlasH/master/doc/images/logo.jpg" alt="SPlisHSPlasH Logo" height="80"/>
</p>

> Paquete de integración para simulaciones de fluidos con **SPlisHSPlasH** en entornos **ROS/Gazebo**.

---

## 📋 Tabla de Contenidos

* [Requisitos Previos](#-requisitos-previos)
* [Instalación](#-instalación)
* [Configuración y Uso](#-configuración-y-uso)
* [Ejemplos de Simulación](#-ejemplos-de-simulación)
* [Mantenimiento](#-mantenimiento)
* [Estructura del Proyecto](#-estructura-del-proyecto)
* [Licencia](#licencia)

---

## 📌 Requisitos Previos

* **Sistema Operativo:** Ubuntu 20.04 LTS

> El resto de componentes se instalan automáticamente mediante los scripts listados a continuación.

---

## 🚀 Instalación

Clona el repositorio y ejecuta los scripts en el siguiente orden:

```bash
cd ~/
git clone https://github.com/ismael-pacheco/ros_splishsplash_integration.git
cd ros_splishsplash_integration/scripts/

./install_base.sh
./install_ignition_math.sh
./install_ruby.sh
./install_sdformat.sh
./install_gazebo.sh
./install_ros.sh
./install_iq_sim.sh
./install_iq_gnc.sh
./configure_ros_gazebo_local.sh
./manage_bashrc_config.sh
./install_splishsplash.sh
./install_px4.sh
```

> Cada script se encarga de instalar o configurar su componente correspondiente.

---

## ⚙️ Configuración y Uso

### Setup del Entorno

Para copiar los archivos de prueba y scripts adicionales a sus rutas adecuadas, ejecuta:

```bash
./scripts/setup_environment.sh
```

Esto copiará:

* **Escenarios Gazebo** (`.sdf`) desde `test files/gazebo/` a `~/gazebo/`
* **Scripts de IQ\_GNC** desde `test files/iq_gnc/scripts/` a `~/catkin_ws/src/iq_gnc/scripts/`
* **Código fuente de IQ\_GNC** desde `test files/iq_gnc/src/` a `~/catkin_ws/src/iq_gnc/src/`
* **Launch files de PX4-Autopilot** desde `test files/PX4-Autopilot/launch/` a `~/PX4-Autopilot/launch/`

---

## 🧪 Ejemplos de Simulación

* **Pool con caja que cae:** `gazebo/pool_with_falling_box.sdf`
* **Rotura de presa en caja:** `gazebo/BoxDamBreak.sdf`
* \*\*Ejecuta una simulación desde su directorio con el comando: \*\* `gazebo pool_with_falling_box.sdf -g libFluidVisPlugin.so`&#x20;

Puedes lanzar estos ejemplos con el mismo launch:

```bash
roslaunch splishsplash_integration fluid_world.launch world:=<ruta_al_sdf>
```

---

## 🔧 Mantenimiento

* **Actualizar paquetes de sistema:**

  ```bash
  ./scripts/update_packages.sh
  ```
* **Actualizar repositorios locales:**

  ```bash
  ./scripts/update_repo.sh
  ```
* **Reconfigurar entorno tras cambios:**

  ```bash
  ./scripts/configure_ros_gazebo_local.sh
  ```

---

## 📁 Estructura del Proyecto

```
ros_splishsplash_integration/
├── scripts/                  # Scripts de instalación y configuración
├── src/                      # Plugins y nodos ROS
├── test files/               # Archivos de prueba (.sdf)
├── README.md
└── LICENSE
```

---

## 📝 Licencia

Este proyecto está bajo la licencia **MIT**.

---

### ⚠️ Notas Importantes

* Ejecutar `source ~/.bashrc` tras cualquier instalación o actualización.

