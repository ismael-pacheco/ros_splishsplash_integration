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

* **Escenarios Gazebo** (`.sdf`) desde `test files/gazebo/` a `ros_splishsplash_integration/launch/worlds/`
* **Scripts de IQ\_GNC** desde `test files/iq_gnc/scripts/` a `~/catkin_ws/src/iq_gnc/scripts/`
* **Código fuente de IQ\_GNC** desde `test files/iq_gnc/src/` a `~/catkin_ws/src/iq_gnc/src/`
* **Launch files de PX4-Autopilot** desde `test files/PX4-Autopilot/launch/` a `~/PX4-Autopilot/launch/`

---

## 🧪 Ejemplos de Simulación

* **Pool con caja que cae:** `gazebo/pool_with_falling_box.sdf`
* **Rotura de presa en caja:** `gazebo/BoxDamBreak.sdf`
* **Ejecuta una simulación desde su directorio con el comando:** `gazebo pool_with_falling_box.sdf -g libFluidVisPlugin.so`

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
│   ├── install_*.sh         # Instaladores de dependencias y componentes
│   ├── configure_*.sh       # Configuración de entorno y ROS/Gazebo local
│   └── update_*.sh          # Actualización de paquetes y repositorios
├── test files/               # Archivos de prueba originales
│   ├── gazebo/               # Escenarios .sdf
│   ├── iq_gnc/               # Código y scripts de IQ_GNC
│   └── PX4-Autopilot/        # Launch files de PX4-Autopilot
├── README.md
└── LICENSE
```

> El workspace Catkin se crea en `~/catkin_ws` e incluye el paquete `iq_gnc` tras ejecutar los instaladores correspondientes.

---


## 🚩 Ejecución de Archivos de Prueba

### Control Avanzado

1. **Lanzar SITL**

   ```bash
   ./startsitl.sh
   ```
2. **Iniciar simulación de pista**

   ```bash
   roslaunch iq_sim runway.launch
   ```
3. **Lanzar ArduPilot/MAVROS**

   ```bash
   roslaunch iq_sim apm.launch
   ```

   Espera a que las IMUs cambien a modo GPS y luego cambia a `GUIDED`.
4. **Ejecutar nodos de control y gráficas**

   ```bash
   rosrun iq_gnc circleArdu
   rosrun iq_gnc plot_adaptive_and_trajectory.py
   ```

### Control SMA (Swarm Multi-Agent)

```bash
cd ~/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic
reset
roslaunch px4 multi_uav_custom.launch
```

En otra terminal:

```bash
roslaunch iq_gnc multi_agent.launch
```

Y en otra:

```bash
rosrun iq_gnc error_plotter.py
```

---

## 🛠️ Modificación de CMakeLists.txt

En el `CMakeLists.txt` de `iq_gnc`, asegúrate de agregar los ejecutables de los nodos de forma individual, por ejemplo:

```cmake
## Add executables for follower nodes
add_executable(follower_node1 src/follower_node1.cpp)
add_dependencies(follower_node1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(follower_node1 ${catkin_LIBRARIES})

add_executable(follower_node2 src/follower_node2.cpp)
add_dependencies(follower_node2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(follower_node2 ${catkin_LIBRARIES})

add_executable(follower_node3 src/follower_node3.cpp)
add_dependencies(follower_node3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(follower_node3 ${catkin_LIBRARIES})

add_executable(follower_node4 src/follower_node4.cpp)
add_dependencies(follower_node4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(follower_node4 ${catkin_LIBRARIES})

## Opcional: Trajectory nodedd_executable(circle_node src/circle.cpp)
add_dependencies(circle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(circle_node ${catkin_LIBRARIES})

## Opcional: Formation monitor
add_executable(formation_monitor src/formation_monitor.cpp)
add_dependencies(formation_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(formation_monitor ${catkin_LIBRARIES})

## Instalar los scripts Python (plot_adaptive_and_trajectory.py) como ejecutable ROS
catkin_install_python(PROGRAMS
  scripts/plot_adaptive_and_trajectory.py
  scripts/error_plotter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

```

