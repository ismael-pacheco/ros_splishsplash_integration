// circle_ardupilot_setpoint_control.cpp - VERSIÓN CORREGIDA
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/StatusText.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <gnc_functions.hpp>
#include <cmath>
#include <string>
#include <sstream>
#include <algorithm>

// Publishers globales
ros::Publisher pub_error_adapt;
ros::Publisher pub_inertia_diff;
ros::Publisher pub_done;
ros::Publisher pub_trajectory_error;
ros::Publisher pub_desired_pose;
ros::Publisher pub_mode_switch;

// Variables globales para almacenar posición actual
geometry_msgs::PoseStamped current_pose;
mavros_msgs::State current_state;
bool pose_received = false;
bool state_received = false;

// Variables para detectar cambio de modo
bool adaptive_mode_active = false;

// Callback para obtener posición actual
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
    pose_received = true;
}

// Callback para obtener estado del drone
void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
    state_received = true;
}

// Función para verificar si el drone ha alcanzado el setpoint
bool hasReachedSetpoint(float target_x, float target_y, float target_z, float tolerance = 0.5f) {
    if (!pose_received) return false;
    
    float dx = target_x - current_pose.pose.position.x;
    float dy = target_y - current_pose.pose.position.y;
    float dz = target_z - current_pose.pose.position.z;
    float distance = sqrt(dx*dx + dy*dy + dz*dz);
    
    return distance < tolerance;
}

// Función para esperar hasta alcanzar setpoint con timeout
bool waitForSetpoint(float target_x, float target_y, float target_z, 
                    float tolerance = 0.5f, float timeout = 10.0f) {
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(20.0);
    
    while (ros::ok()) {
        ros::spinOnce();
        
        // Verificar timeout
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_WARN("Timeout esperando setpoint (%.2f, %.2f, %.2f)", target_x, target_y, target_z);
            return false;
        }
        
        // Continuar publicando el setpoint mientras esperamos
        set_destination(target_x, target_y, target_z, 0);
        
        // Verificar si hemos llegado
        if (hasReachedSetpoint(target_x, target_y, target_z, tolerance)) {
            ROS_INFO("Setpoint alcanzado: (%.2f, %.2f, %.2f)", target_x, target_y, target_z);
            return true;
        }
        
        rate.sleep();
    }
    return false;
}

// Callback para STATUSTEXT: parsea mensajes del controlador adaptativo
void statusTextCallback(const mavros_msgs::StatusText::ConstPtr& msg) {
    const std::string& line = msg->text;
    
    // Debug: mostrar todos los mensajes STATUSTEXT
    ROS_DEBUG_STREAM("STATUSTEXT: " << line);
    
    // Parsear errores de adaptación: "ADAPT_ERROR e=x,y,z"
    const std::string prefix_adapt = "ADAPT_ERROR e=";
    if (line.find(prefix_adapt) == 0) {
        float ex, ey, ez;
        std::string numbers = line.substr(prefix_adapt.size());
        std::replace(numbers.begin(), numbers.end(), ',', ' ');
        std::stringstream ss(numbers);
        if (ss >> ex >> ey >> ez) {
            geometry_msgs::Vector3Stamped v_msg;
            v_msg.header.stamp = ros::Time::now();
            v_msg.header.frame_id = "base_link";
            v_msg.vector.x = ex;
            v_msg.vector.y = ey;
            v_msg.vector.z = ez;
            pub_error_adapt.publish(v_msg);
            ROS_DEBUG("Publicado error adaptativo: %.3f, %.3f, %.3f", ex, ey, ez);
        }
        return;
    }

    // Parsear diferencias de inercia: "INERTIA_DIFF d=x,y,z"
    const std::string prefix_inertia = "INERTIA_DIFF d=";
    if (line.find(prefix_inertia) == 0) {
        float dxx, dyy, dzz;
        std::string numbers = line.substr(prefix_inertia.size());
        std::replace(numbers.begin(), numbers.end(), ',', ' ');
        std::stringstream ss(numbers);
        if (ss >> dxx >> dyy >> dzz) {
            geometry_msgs::Vector3Stamped v_msg;
            v_msg.header.stamp = ros::Time::now();
            v_msg.header.frame_id = "base_link";
            v_msg.vector.x = dxx;
            v_msg.vector.y = dyy;
            v_msg.vector.z = dzz;
            pub_inertia_diff.publish(v_msg);
            ROS_DEBUG("Publicado diferencia inercia: %.3f, %.3f, %.3f", dxx, dyy, dzz);
        }
        return;
    }

    // Parsear cambio de modo: "MODE_SWITCH adaptive=1" o "MODE_SWITCH adaptive=0"
    const std::string prefix_mode = "MODE_SWITCH adaptive=";
    if (line.find(prefix_mode) == 0) {
        std::string mode_str = line.substr(prefix_mode.size());
        try {
            int mode_value = std::stoi(mode_str);
            bool new_adaptive_mode = (mode_value == 1);
            
            // Solo publicar si hay cambio de modo
            if (new_adaptive_mode != adaptive_mode_active) {
                adaptive_mode_active = new_adaptive_mode;
                
                std_msgs::Bool mode_msg;
                mode_msg.data = adaptive_mode_active;
                pub_mode_switch.publish(mode_msg);
                
                if (adaptive_mode_active) {
                    ROS_INFO("🔄 Controlador cambió a MODO ADAPTATIVO");
                } else {
                    ROS_WARN("⚠️  Controlador cambió a MODO PID");
                }
            }
        } catch (const std::exception& e) {
            ROS_WARN("Error parseando MODE_SWITCH: %s", e.what());
        }
        return;
    }

    // Detectar otros mensajes importantes del controlador adaptativo
    if (line.find("Adapt") != std::string::npos || 
        line.find("ADAPT") != std::string::npos ||
        line.find("adaptive") != std::string::npos) {
        ROS_INFO("Mensaje adaptativo detectado: %s", line.c_str());
        
        // Si detectamos actividad adaptativa pero no hemos recibido MODE_SWITCH,
        // asumir que está en modo adaptativo
        if (!adaptive_mode_active) {
            adaptive_mode_active = true;
            std_msgs::Bool mode_msg;
            mode_msg.data = true;
            pub_mode_switch.publish(mode_msg);
            ROS_INFO("🔄 Modo adaptativo inferido de mensaje: %s", line.c_str());
        }
    }
    
    // Detectar mensajes de transición importantes
    if (line.find("TAKEOFF_COMPLETE") != std::string::npos) {
        ROS_INFO("Despegue completado - Iniciando modo adaptativo");
    }
    
    if (line.find("CRITICAL: Reverting to PID") != std::string::npos || 
        line.find("Reverting to PID") != std::string::npos) {
        ROS_WARN("Condición crítica detectada - Regresando a control PID");
        // Forzar cambio a modo PID
        if (adaptive_mode_active) {
            adaptive_mode_active = false;
            std_msgs::Bool mode_msg;
            mode_msg.data = false;
            pub_mode_switch.publish(mode_msg);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_ardupilot_setpoint_control");
    ros::NodeHandle nh("~");

    // Configurar nivel de log para debug si es necesario
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // 1) Inicializar publicadores/suscriptores estándar
    init_publisher_subscriber(nh);

    // 2) Publicadores para datos de adaptación y monitoreo
    pub_error_adapt = nh.advertise<geometry_msgs::Vector3Stamped>("/adaptive/error", 10);
    pub_inertia_diff = nh.advertise<geometry_msgs::Vector3Stamped>("/adaptive/inertia_diff", 10);
    pub_mode_switch = nh.advertise<std_msgs::Bool>("/adaptive/mode_switch", 10, true);
    pub_done = nh.advertise<std_msgs::Bool>("/adaptive/done", 1, true);
    pub_trajectory_error = nh.advertise<geometry_msgs::Vector3Stamped>("/trajectory/position_error", 10);
    pub_desired_pose = nh.advertise<geometry_msgs::PoseStamped>("/desired/pose", 10);

    // 3) Suscriptores
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10, poseCallback);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, stateCallback);
    ros::Subscriber statustext_sub = nh.subscribe<mavros_msgs::StatusText>(
        "/mavros/statustext/recv", 100, statusTextCallback);

    // 4) Esperar conexión con FCU
    ROS_INFO("Esperando conexión con FCU...");
    wait4connect();

    // 5) Inicializar marco local
    ROS_INFO("Inicializando marco de referencia local...");
    initialize_local_frame();
    
    // Esperar a recibir datos del estado y posición
    ROS_INFO("Esperando datos del drone...");
    while (ros::ok() && (!pose_received || !state_received)) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // 6) Verificar que estamos en modo GUIDED
    if (current_state.mode != "GUIDED") {
        ROS_INFO("Cambiando a modo GUIDED...");
        set_mode("GUIDED");
        ros::Duration(2.0).sleep();
    }

    // 7) Despegar si no estamos en el aire
    ROS_INFO("Iniciando despegue a 3m...");
    takeoff(3);
    
    // Esperar a alcanzar altitud de despegue
    ROS_INFO("Esperando alcanzar altitud de despegue...");
    bool takeoff_complete = waitForSetpoint(0, 0, 3, 0.5f, 15.0f);
    if (!takeoff_complete) {
        ROS_ERROR("Fallo en despegue - abortando misión");
        return -1;
    }

    ROS_INFO("Despegue completado. Posición actual: (%.2f, %.2f, %.2f)", 
             current_pose.pose.position.x, 
             current_pose.pose.position.y, 
             current_pose.pose.position.z);

    // 8) Parámetros para trayectoria circular
    const float radius = 5.0f;
    const float center_x = current_pose.pose.position.x;  // Usar posición actual como centro
    const float center_y = current_pose.pose.position.y;
    const float base_z = 3.0f;
    const float z_amplitude = 1.0f;
    const int num_waypoints = 20;  // 12 puntos = cada 30 grados
    const float total_circles = 2.0f;
    
    // Publicar estado inicial del modo (PID)
    std_msgs::Bool initial_mode;
    initial_mode.data = false;
    pub_mode_switch.publish(initial_mode);
    ROS_INFO("Estado inicial: MODO PID");

    ROS_INFO("Iniciando trayectoria circular con %d waypoints por círculo", num_waypoints);
    ROS_INFO("Centro del círculo: (%.2f, %.2f)", center_x, center_y);

    // 9) Bucle principal - control por waypoints discretos
    for (int circle = 0; circle < (int)total_circles && ros::ok(); circle++) {
        ROS_INFO("=== CÍRCULO %d/%d ===", circle + 1, (int)total_circles);
        
        for (int wp = 0; wp < num_waypoints && ros::ok(); wp++) {
            float angle = (2.0f * M_PI * wp) / num_waypoints;
            
            // Calcular posición objetivo
            float target_x = center_x + radius * cos(angle);
            float target_y = center_y + radius * sin(angle);
            float target_z = base_z + z_amplitude * sin(angle * 2.0f);
            
            // Calcular heading tangente al círculo
            float desired_heading = angle + M_PI/2.0f;
            while (desired_heading > M_PI) desired_heading -= 2.0f * M_PI;
            while (desired_heading < -M_PI) desired_heading += 2.0f * M_PI;
            
            ROS_INFO("Waypoint %d/%d: (%.2f, %.2f, %.2f)", 
                     wp + 1, num_waypoints, target_x, target_y, target_z);
            
            // Publicar posición deseada para monitoreo
            geometry_msgs::PoseStamped desired_pose_msg;
            desired_pose_msg.header.stamp = ros::Time::now();
            desired_pose_msg.header.frame_id = "base_link";
            desired_pose_msg.pose.position.x = target_x;
            desired_pose_msg.pose.position.y = target_y;
            desired_pose_msg.pose.position.z = target_z;
            desired_pose_msg.pose.orientation.w = cos(desired_heading / 2.0f);
            desired_pose_msg.pose.orientation.z = sin(desired_heading / 2.0f);
            pub_desired_pose.publish(desired_pose_msg);
            
            // Moverse al waypoint y esperar a llegar
            bool reached = waitForSetpoint(target_x, target_y, target_z, 0.8f, 15.0f);
            
            if (!reached) {
                ROS_WARN("No se pudo alcanzar waypoint %d - continuando", wp + 1);
            }
            
            // Calcular y publicar errores de trayectoria
            if (pose_received) {
                float ex = target_x - current_pose.pose.position.x;
                float ey = target_y - current_pose.pose.position.y;
                float ez = target_z - current_pose.pose.position.z;
                
                geometry_msgs::Vector3Stamped error_msg;
                error_msg.header.stamp = ros::Time::now();
                error_msg.header.frame_id = "base_link";
                error_msg.vector.x = ex;
                error_msg.vector.y = ey;
                error_msg.vector.z = ez;
                pub_trajectory_error.publish(error_msg);
                
                ROS_INFO("Error actual: (%.2f, %.2f, %.2f) | Modo: %s",
                         ex, ey, ez, adaptive_mode_active ? "ADAPTATIVO" : "PID");
            }
            
            // Pausa breve entre waypoints
            ros::Duration(0.5).sleep();
        }
    }

    // 10) Regresar al centro y mantener posición
    ROS_INFO("Regresando al centro para finalizar...");
    waitForSetpoint(center_x, center_y, base_z, 0.5f, 10.0f);
    
    // Mantener posición final
    ROS_INFO("Manteniendo posición final...");
    ros::Rate hold_rate(10.0);
    for (int i = 0; i < 30 && ros::ok(); ++i) {  // 3 segundos
        set_destination(center_x, center_y, base_z, 0);
        ros::spinOnce();
        hold_rate.sleep();
    }

    // 11) Indicar finalización
    std_msgs::Bool done_msg;
    done_msg.data = true;
    pub_done.publish(done_msg);
    ros::spinOnce();
    
    ROS_INFO("Trayectoria completada exitosamente.");
    
    // 12) Aterrizaje seguro
    ROS_INFO("Iniciando secuencia de aterrizaje...");
    set_mode("RTL");
    ros::Duration(10).sleep();
    set_mode("LAND");
    ros::Duration(10).sleep();
    
    ROS_INFO("Misión completada.");
    return 0;
}