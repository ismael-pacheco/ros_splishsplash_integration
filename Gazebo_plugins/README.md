# Rotor Wrench Plugin para Gazebo

Plugin de Gazebo que permite controlar un dron mediante mensajes ROS de tipo `WrenchStamped`, aplicando fuerzas y torques directamente a rotores individuales sin necesidad de PX4 o ArduPilot.

## Características

- ✅ Control directo de 8 rotores mediante topics ROS
- ✅ Aplicación de fuerzas y torques en posiciones específicas
- ✅ Thread dedicado para callbacks de ROS (sin bloqueo del loop de simulación)
- ✅ Compatible con Gazebo Classic 11.x y ROS Noetic
- ✅ Debug configurable para monitoreo en tiempo real

## Requisitos del Sistema

- **OS**: Ubuntu 20.04
- **ROS**: Noetic
- **Gazebo**: Classic 11.x
- **Dependencias**:
  - `ros-noetic-gazebo-ros`
  - `ros-noetic-geometry-msgs`
  - `ros-noetic-roscpp`
  - CMake >= 3.0.2

## Estructura del Proyecto

```
rotor_wrench_plugin/
├── CMakeLists.txt           # Configuración de compilación
├── package.xml              # Definición del paquete ROS
├── RotorWrenchPlugin.cc     # Código fuente del plugin
├── model.sdf                # Modelo del dron (MightyMorphinDrone)
├── README.md                # Esta documentación
└── setup.sh                 # Script de instalación automática
```

## Instalación

### Opción 1: Script Automático (Recomendado)

```bash
cd rotor_wrench_plugin
chmod +x setup.sh
./setup.sh
```

### Opción 2: Manual

```bash
# 1. Instalar dependencias
sudo apt update
sudo apt install ros-noetic-gazebo-ros ros-noetic-geometry-msgs ros-noetic-roscpp

# 2. Compilar el plugin
mkdir -p build
cd build
cmake ..
make

# 3. Configurar variables de entorno (agregar a ~/.bashrc)
echo "export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:$(pwd)" >> ~/.bashrc
source ~/.bashrc

# 4. Configurar el modelo de Gazebo
mkdir -p ~/.gazebo/models/Drone
cp model.sdf ~/.gazebo/models/Drone/
# Copiar también las mallas (meshes/Drone.dae) si las tienes
```

## Uso

### 1. Iniciar Gazebo

```bash
# Con modo verbose para ver mensajes del plugin
gazebo --verbose

# O sin verbose para uso normal
gazebo
```

### 2. Insertar el Modelo

- En Gazebo, ve a la pestaña "Insert"
- Busca el modelo "MightyMorphinDrone"
- Haz clic para insertarlo en el mundo

### 3. Publicar Comandos de Wrench

El plugin crea automáticamente 8 topics ROS, uno por cada rotor:

```bash
/drone/rotor_side_pos/wrench
/drone/rotor_side_neg/wrench
/drone/rotor_side_pos_top/wrench
/drone/rotor_side_neg_top/wrench
/drone/rotor_front_pos/wrench
/drone/rotor_front_neg/wrench
/drone/rotor_back_pos/wrench
/drone/rotor_back_neg/wrench
```

#### Ejemplo: Aplicar fuerza a un rotor

```bash
rostopic pub /drone/rotor_front_pos/wrench geometry_msgs/WrenchStamped \
  '{wrench: {force: {x: 0, y: 0, z: 50}, torque: {x: 0, y: 0, z: 0}}}' -r 10
```

#### Ejemplo: Hovering balanceado

```bash
# Aplicar ~6.5N a cada rotor para contrarrestar gravedad (~5kg × 9.81 / 8 rotores)
for rotor in rotor_side_pos rotor_side_neg rotor_side_pos_top rotor_side_neg_top \
             rotor_front_pos rotor_front_neg rotor_back_pos rotor_back_neg; do
  rostopic pub /drone/$rotor/wrench geometry_msgs/WrenchStamped \
    '{wrench: {force: {x: 0, y: 0, z: 6.5}, torque: {x: 0, y: 0, z: 0}}}' -r 10 &
done
```

Para detener todos los publishers:
```bash
killall rostopic
```

## Arquitectura del Plugin

### Flujo de Datos

```
ROS Topics → CallbackQueue → Thread ROS → Mutex → OnUpdate() → Gazebo Physics
```

1. **Suscripción**: El plugin se suscribe a 8 topics de tipo `WrenchStamped`
2. **Callback Thread**: Un thread separado procesa los mensajes entrantes
3. **Almacenamiento**: Los últimos comandos se guardan en estructuras `Rotor`
4. **Aplicación**: En cada update de Gazebo, se aplican las fuerzas al `base_link`

### Configuración de Rotores

Los rotores están posicionados según la geometría del dron:

| Rotor | Posición (x, y, z) |
|-------|-------------------|
| `rotor_side_pos` | (0, 0.414, -0.040) |
| `rotor_side_neg` | (0, -0.414, -0.040) |
| `rotor_side_pos_top` | (0, 0.414, 0.040) |
| `rotor_side_neg_top` | (0, -0.414, 0.040) |
| `rotor_front_pos` | (0.331, 0, -0.040) |
| `rotor_front_neg` | (0.331, 0, 0.040) |
| `rotor_back_pos` | (-0.346, 0, -0.040) |
| `rotor_back_neg` | (-0.346, 0, 0.040) |

## Parámetros Físicos

- **Masa del dron**: 4.987 kg
- **Fuerza de gravedad**: 48.9 N (masa × 9.81 m/s²)
- **Fuerza por rotor (hovering)**: ~6.1 N
- **Momentos de inercia**: Ver `model.sdf`

## Verificación y Debug

### Verificar que el plugin está cargado

```bash
# Listar topics ROS
rostopic list | grep wrench

# Ver información de un topic
rostopic info /drone/rotor_front_pos/wrench
```

### Verificar suscriptores

Deberías ver `/rotor_wrench_plugin` como subscriber:
```bash
rostopic info /drone/rotor_front_pos/wrench
# Subscribers: 
#  * /rotor_wrench_plugin (http://...)
```

### Monitorear mensajes recibidos

Con Gazebo en modo `--verbose`, verás:
```
[Msg] [RotorWrenchPlugin] Recibido mensaje en rotor_front_pos - Fuerza: [0, 0, 50]
[Msg] [RotorWrenchPlugin] Pos: [...] Vel: [...]
```

## Integración con Controladores

### Ejemplo en Python

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        
        # Publishers para cada rotor
        self.pubs = {
            'rotor_side_pos': rospy.Publisher('/drone/rotor_side_pos/wrench', 
                                             WrenchStamped, queue_size=10),
            'rotor_side_neg': rospy.Publisher('/drone/rotor_side_neg/wrench', 
                                             WrenchStamped, queue_size=10),
            # ... (crear para todos los rotores)
        }
        
    def apply_wrench(self, rotor_name, fx, fy, fz, tx, ty, tz):
        msg = WrenchStamped()
        msg.wrench.force.x = fx
        msg.wrench.force.y = fy
        msg.wrench.force.z = fz
        msg.wrench.torque.x = tx
        msg.wrench.torque.y = ty
        msg.wrench.torque.z = tz
        self.pubs[rotor_name].publish(msg)
    
    def hover(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            for rotor in self.pubs.keys():
                self.apply_wrench(rotor, 0, 0, 6.5, 0, 0, 0)
            rate.sleep()

if __name__ == '__main__':
    controller = DroneController()
    controller.hover()
```

### Ejemplo en C++

```cpp
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_controller");
    ros::NodeHandle nh;
    
    std::vector<std::string> rotors = {
        "rotor_side_pos", "rotor_side_neg", 
        "rotor_side_pos_top", "rotor_side_neg_top",
        "rotor_front_pos", "rotor_front_neg",
        "rotor_back_pos", "rotor_back_neg"
    };
    
    std::map<std::string, ros::Publisher> pubs;
    for (const auto& rotor : rotors) {
        pubs[rotor] = nh.advertise<geometry_msgs::WrenchStamped>(
            "/drone/" + rotor + "/wrench", 10);
    }
    
    ros::Rate rate(10);
    while (ros::ok()) {
        geometry_msgs::WrenchStamped msg;
        msg.wrench.force.z = 6.5;  // Hovering force
        
        for (const auto& pair : pubs) {
            pair.second.publish(msg);
        }
        
        rate.sleep();
    }
    
    return 0;
}
```

## Troubleshooting

### El plugin no se carga

**Síntoma**: No aparecen los topics `/drone/rotor_*/wrench`

**Solución**:
```bash
# Verificar que la librería existe
ls -lh build/libRotorWrenchPlugin.so

# Verificar variable de entorno
echo $GAZEBO_PLUGIN_PATH

# Agregar al path si no está
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/ruta/a/build
```

### El dron no se mueve

**Síntoma**: Los topics existen pero el dron no responde

**Verificar**:
1. Que Gazebo esté en modo `--verbose` para ver logs
2. Que los mensajes se estén recibiendo (buscar "Recibido mensaje" en logs)
3. Que las fuerzas sean suficientes (>6N por rotor para hovering)
4. Que el dron no esté colisionando con el suelo

**Prueba de diagnóstico**:
```bash
# Desactivar gravedad temporalmente en Gazebo: World → Physics → gravity Z = 0
# Aplicar fuerza horizontal
rostopic pub /drone/rotor_front_pos/wrench geometry_msgs/WrenchStamped \
  '{wrench: {force: {x: 100, y: 0, z: 0}, torque: {x: 0, y: 0, z: 0}}}' -r 10
```

### Error de compilación

**Síntoma**: CMake o make fallan

**Solución**:
```bash
# Reinstalar dependencias
sudo apt install --reinstall ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# Limpiar y recompilar
cd build
rm -rf *
cmake ..
make
```

### Advertencia sobre callbacks

**Síntoma**: `Warning: Deleting a connection right after creation`

**Causa**: Normal en Gazebo, no afecta la funcionalidad

## Limitaciones Conocidas

- ⚠️ No modela aerodinámicamente las hélices (solo fuerzas/torques directos)
- ⚠️ No incluye efectos de ground effect
- ⚠️ Las esferas visuales de los rotores son decorativas (sin física propia)
- ⚠️ Requiere spawn manual del modelo (no incluye launch file)

## Mejoras Futuras

- [ ] Launch file para spawn automático
- [ ] Parámetros configurables desde ROS (posiciones de rotores)
- [ ] Modelo aerodinámico de hélices
- [ ] Simulación de ruido en sensores
- [ ] Interfaz para configuración dinámica

## Contribuir

Para reportar bugs o sugerir mejoras, por favor:
1. Describe el problema claramente
2. Incluye logs de Gazebo (`--verbose`)
3. Especifica versiones de ROS/Gazebo/Ubuntu

## Licencia

MIT License - Ver archivo LICENSE para detalles

## Autores

- Plugin desarrollado para control directo de drones sin autopiloto
- Basado en arquitectura de plugins de Gazebo Classic

## Referencias

- [Gazebo Plugin Tutorial](http://gazebosim.org/tutorials?tut=plugins_model)
- [ROS WrenchStamped Message](http://docs.ros.org/en/api/geometry_msgs/html/msg/WrenchStamped.html)
- [Gazebo Physics API](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/group__gazebo__physics.html)
