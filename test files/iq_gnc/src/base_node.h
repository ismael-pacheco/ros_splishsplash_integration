#ifndef BASE_NODE_H
#define BASE_NODE_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <map>
#include <vector>

class BaseNode {
protected:

    ros::NodeHandle nh_;
    std::string ns_;
    int drone_id_;
    
    // Publishers
    ros::Publisher individual_metrics_pub_;
    ros::Publisher setpoint_pub_;
    ros::Publisher ready_pub_;
    ros::Publisher virtual_pose_pub_;
    ros::Publisher consensus_error_pub_;
    ros::Publisher trajectory_error_pub_;
    ros::Publisher neighbor_pose_pub_;
    ros::Publisher planned_trajectory_pub_;
    ros::Publisher formation_debug_pub_;  // *** ADDED: For formation debugging ***
    
    // Subscribers
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber neighbor_pose_sub_;
    ros::Subscriber ack_sub_;
    ros::Subscriber planned_trajectory_sub_;
    
    // Service clients
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    
    // State variables
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped virtual_agent_pose_;
    geometry_msgs::PoseStamped formation_center_;
    geometry_msgs::PoseStamped desired_trajectory_pose_;
    geometry_msgs::PoseStamped planned_trajectory_point_;
    
    bool formation_ready_;
    bool ack_received_;
    bool takeoff_completed_;
    bool planned_trajectory_received_;
    
    // Formation parameters
    double formation_radius_;
    double formation_height_;
    double consensus_gain_;
    double control_gain_;
    double neighbor_timeout_;
    int total_drones_;  // *** ADDED: Total number of drones in formation ***
    
    // *** MODIFIED: Circle formation offsets (dynamic based on total_drones) ***
    std::vector<std::pair<double, double>> circle_offsets_;
    
    // Virtual agent consensus variables
    geometry_msgs::Point virtual_agent_center_;
    geometry_msgs::Point desired_center_;
    
    // Neighbor tracking for consensus (virtual centers of neighbors)
    std::map<int, geometry_msgs::Point> neighbor_virtual_centers_;
    std::map<int, ros::Time> last_neighbor_updates_;
    
    // Error tracking
    double current_consensus_error_;
    double current_trajectory_error_;
    double current_formation_error_;  // *** ADDED: Formation distance error ***
    
    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }
    
    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        calculateFormationTrajectoryError();
        calculateFormationDistanceError();  // *** ADDED: Calculate formation distance error ***
    }
    
    void neighborPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // Extract drone ID from frame_id
        std::string frame_id = msg->header.frame_id;
        if (frame_id.find("uav") != std::string::npos) {
            size_t uav_pos = frame_id.find("uav");
            int neighbor_id = std::stoi(frame_id.substr(uav_pos + 3, 1));
            
            if (neighbor_id != drone_id_) {
                // Store neighbor's virtual center
                neighbor_virtual_centers_[neighbor_id] = msg->pose.position;
                last_neighbor_updates_[neighbor_id] = ros::Time::now();
            }
        }
        cleanOldNeighbors();
    }
    
    void plannedTrajectoryCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        planned_trajectory_point_ = *msg;
        planned_trajectory_received_ = true;
        calculateFormationTrajectoryError();
        
        ROS_DEBUG("%s: Planned trajectory received: (%.2f, %.2f, %.2f)", 
                 ns_.c_str(), msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }
    
    void ackCb(const std_msgs::Bool::ConstPtr& msg) {
        ack_received_ = msg->data;
        if (ack_received_) {
            ROS_INFO("%s: ACK recibido", ns_.c_str());
        }
    }
    
    // *** MODIFIED: Initialize circle offsets based on total number of drones ***
    void initializeCircleOffsets() {
        circle_offsets_.clear();
        
        // Ensure we have at least 1 drone and maximum reasonable number
        if (total_drones_ < 1) total_drones_ = 1;
        if (total_drones_ > 20) total_drones_ = 20;
        
        for (int i = 0; i < total_drones_; ++i) {
            // Calculate angle for uniform distribution around circle
            double angle = 2.0 * M_PI * i / static_cast<double>(total_drones_);
            
            // All drones at exactly the same distance (formation_radius_)
            double offset_x = formation_radius_ * cos(angle);
            double offset_y = formation_radius_ * sin(angle);
            
            circle_offsets_.emplace_back(offset_x, offset_y);
            
            ROS_DEBUG("%s: Drone %d - Angle: %.2f rad (%.1f deg), Offset: (%.2f, %.2f)", 
                     ns_.c_str(), i, angle, angle * 180.0 / M_PI, offset_x, offset_y);
        }
        
        ROS_INFO("%s: Circle formation initialized - %d drones, radius %.2f m", 
                 ns_.c_str(), total_drones_, formation_radius_);
    }
    
    void cleanOldNeighbors() {
        ros::Time now = ros::Time::now();
        for (auto it = neighbor_virtual_centers_.begin(); it != neighbor_virtual_centers_.end(); ) {
            auto update_it = last_neighbor_updates_.find(it->first);
            if (update_it == last_neighbor_updates_.end() || 
                (now - update_it->second).toSec() > neighbor_timeout_) {
                ROS_WARN("%s: Removing stale neighbor %d", ns_.c_str(), it->first);
                last_neighbor_updates_.erase(it->first);
                neighbor_virtual_centers_.erase(it++);
            } else {
                ++it;
            }
        }
    }
    
    void calculateFormationTrajectoryError() {
        if (!planned_trajectory_received_ || planned_trajectory_point_.header.stamp.isZero()) {
            current_trajectory_error_ = 0.0;
            return;
        }
        
        // Calculate error between virtual agent center (formation center) and planned trajectory point
        double dx = virtual_agent_center_.x - planned_trajectory_point_.pose.position.x;
        double dy = virtual_agent_center_.y - planned_trajectory_point_.pose.position.y;
        double dz = virtual_agent_center_.z - planned_trajectory_point_.pose.position.z;
        
        current_trajectory_error_ = sqrt(dx*dx + dy*dy + dz*dz);
        
        // Publish trajectory error
        std_msgs::Float32 traj_error_msg;
        traj_error_msg.data = current_trajectory_error_;
        trajectory_error_pub_.publish(traj_error_msg);
        
        ROS_DEBUG("%s: Formation trajectory error: %.3f (Virtual: (%.2f,%.2f,%.2f) vs Planned: (%.2f,%.2f,%.2f))", 
                 ns_.c_str(), current_trajectory_error_,
                 virtual_agent_center_.x, virtual_agent_center_.y, virtual_agent_center_.z,
                 planned_trajectory_point_.pose.position.x, planned_trajectory_point_.pose.position.y, planned_trajectory_point_.pose.position.z);
    }
    
    // *** ADDED: Calculate formation distance error (how far is drone from desired circle radius) ***
    void calculateFormationDistanceError() {
        // Calculate actual distance from drone to formation center
        double dx = current_pose_.pose.position.x - virtual_agent_center_.x;
        double dy = current_pose_.pose.position.y - virtual_agent_center_.y;
        double actual_distance = sqrt(dx*dx + dy*dy);  // Only XY plane for circular formation
        
        // Formation error: difference between actual distance and desired radius
        current_formation_error_ = std::abs(actual_distance - formation_radius_);
        
        ROS_DEBUG("%s: Formation distance - Actual: %.3f, Desired: %.3f, Error: %.3f", 
                 ns_.c_str(), actual_distance, formation_radius_, current_formation_error_);
    }
    
    void calculateConsensusError() {
        if (neighbor_virtual_centers_.empty()) {
            // No neighbors: error is distance to desired center
            double dx = virtual_agent_center_.x - desired_center_.x;
            double dy = virtual_agent_center_.y - desired_center_.y;
            double dz = virtual_agent_center_.z - desired_center_.z;
            
            current_consensus_error_ = sqrt(dx*dx + dy*dy + dz*dz);
        } else {
            // With neighbors: error is distance from average of all centers
            double avg_x = virtual_agent_center_.x;
            double avg_y = virtual_agent_center_.y;
            double avg_z = virtual_agent_center_.z;
            int count = 1;
            
            for (const auto& neighbor : neighbor_virtual_centers_) {
                avg_x += neighbor.second.x;
                avg_y += neighbor.second.y;
                avg_z += neighbor.second.z;
                count++;
            }
            
            avg_x /= count;
            avg_y /= count;
            avg_z /= count;
            
            // Consensus error: distance from average
            double dx = virtual_agent_center_.x - avg_x;
            double dy = virtual_agent_center_.y - avg_y;
            double dz = virtual_agent_center_.z - avg_z;
            
            current_consensus_error_ = sqrt(dx*dx + dy*dy + dz*dz);
        }
        
        // Publish consensus error
        std_msgs::Float32 consensus_error_msg;
        consensus_error_msg.data = current_consensus_error_;
        consensus_error_pub_.publish(consensus_error_msg);
    }
    
public:
    BaseNode(ros::NodeHandle& nh, const std::string& ns, int drone_id) 
        : nh_(nh), ns_(ns), drone_id_(drone_id), formation_ready_(false), 
          ack_received_(false), takeoff_completed_(false), 
          planned_trajectory_received_(false),
          current_consensus_error_(0.0), current_trajectory_error_(0.0), current_formation_error_(0.0) {
        
        // Initialize parameters
        nh_.param<double>("formation_radius", formation_radius_, 2.0);  // *** MODIFIED: Increased default radius ***
        nh_.param<double>("formation_height", formation_height_, 5.0);
        nh_.param<double>("consensus_gain", consensus_gain_, 0.5);
        nh_.param<double>("control_gain", control_gain_, 0.4);  // *** MODIFIED: Increased for better formation keeping ***
        nh_.param<double>("neighbor_timeout", neighbor_timeout_, 2.0);
        nh_.param<int>("total_drones", total_drones_, 5);  // *** ADDED: Parameter for total number of drones ***
        
        // Validate drone_id
        if (drone_id_ < 0) {
            ROS_WARN("%s: Invalid drone_id %d, setting to 0", ns_.c_str(), drone_id_);
            drone_id_ = 0;
        }
        
        // Initialize circle formation
        initializeCircleOffsets();
        
        // Initialize virtual agent center with small random offset for consensus
        double random_offset = 0.5 * (static_cast<double>(drone_id_) / static_cast<double>(total_drones_) - 0.5);
        virtual_agent_center_.x = random_offset;
        virtual_agent_center_.y = random_offset * 0.5;
        virtual_agent_center_.z = formation_height_;
        
        // Initialize desired center
        desired_center_.x = 0.0;
        desired_center_.y = 0.0;
        desired_center_.z = formation_height_;
        
        // Initialize virtual agent pose
        virtual_agent_pose_ = geometry_msgs::PoseStamped();
        virtual_agent_pose_.pose.position.z = formation_height_;
        
        // Initialize planned trajectory point
        planned_trajectory_point_ = geometry_msgs::PoseStamped();
        planned_trajectory_point_.pose.position.x = 0.0;
        planned_trajectory_point_.pose.position.y = 0.0;
        planned_trajectory_point_.pose.position.z = formation_height_;
        
        // Initialize publishers
        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            ns_ + "/mavros/setpoint_position/local", 10);
        ready_pub_ = nh_.advertise<std_msgs::Bool>(
            ns_ + "/ready", 10);
        virtual_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            ns_ + "/virtual_agent_pose", 10);
        consensus_error_pub_ = nh_.advertise<std_msgs::Float32>(
            "/consensus_error", 10);
        trajectory_error_pub_ = nh_.advertise<std_msgs::Float32>(
            "/trajectory_error", 10);
        neighbor_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/neighbor_poses", 10);
        planned_trajectory_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/planned_trajectory_point", 10);
        formation_debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(  // *** ADDED: Formation debug info ***
            "/formation_debug", 10);
            
        // Initialize subscribers
        state_sub_ = nh_.subscribe<mavros_msgs::State>(
            ns_ + "/mavros/state", 10, &BaseNode::stateCb, this);
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            ns_ + "/mavros/local_position/pose", 10, &BaseNode::poseCb, this);
        neighbor_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/neighbor_poses", 10, &BaseNode::neighborPoseCb, this);
        ack_sub_ = nh_.subscribe<std_msgs::Bool>(
            ns_ + "/ack", 10, &BaseNode::ackCb, this);
        planned_trajectory_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/planned_trajectory_point", 10, &BaseNode::plannedTrajectoryCb, this);
            
        // Initialize service clients
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
            ns_ + "/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
            ns_ + "/mavros/set_mode");
            
        ROS_INFO("%s: BaseNode initialized - Drone ID: %d/%d, Formation radius: %.2f m, Initial virtual center: (%.2f, %.2f, %.2f)", 
                 ns_.c_str(), drone_id_, total_drones_, formation_radius_,
                 virtual_agent_center_.x, virtual_agent_center_.y, virtual_agent_center_.z);
    }
    
    virtual ~BaseNode() {}
    
    bool waitForFCUConnection(ros::Rate& rate) {
        ROS_INFO("%s: Esperando conexion FCU...", ns_.c_str());
        while (ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("%s: FCU conectado", ns_.c_str());
        return true;
    }
    
    void initializeSetpoints(ros::Rate& rate) {
        ROS_INFO("%s: Inicializando setpoints...", ns_.c_str());
        current_pose_.pose.position.z = formation_height_;
        
        // Send initial setpoints for 2 seconds
        ros::Time last_request = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - last_request < ros::Duration(2.0))) {
            setpoint_pub_.publish(current_pose_);
            ros::spinOnce();
            rate.sleep();
        }
    }
    
    bool setOffboardMode(ros::Rate& rate) {
        ROS_INFO("%s: Cambiando a modo OFFBOARD...", ns_.c_str());
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        
        ros::Time last_request = ros::Time::now();
        while (ros::ok() && current_state_.mode != "OFFBOARD") {
            if (ros::Time::now() - last_request > ros::Duration(1.0)) {
                if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("%s: OFFBOARD habilitado", ns_.c_str());
                }
                last_request = ros::Time::now();
            }
            setpoint_pub_.publish(current_pose_);
            ros::spinOnce();
            rate.sleep();
        }
        return current_state_.mode == "OFFBOARD";
    }
    
    bool armVehicle(ros::Rate& rate) {
        ROS_INFO("%s: Armando vehiculo...", ns_.c_str());
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        
        ros::Time last_request = ros::Time::now();
        while (ros::ok() && !current_state_.armed) {
            if (ros::Time::now() - last_request > ros::Duration(1.0)) {
                if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("%s: Vehiculo armado", ns_.c_str());
                }
                last_request = ros::Time::now();
            }
            setpoint_pub_.publish(current_pose_);
            ros::spinOnce();
            rate.sleep();
        }
        return current_state_.armed;
    }
    
    bool takeoff(ros::Rate& rate) {
        ROS_INFO("%s: Iniciando despegue a %.1f metros...", ns_.c_str(), formation_height_);
        geometry_msgs::PoseStamped takeoff_pose = current_pose_;
        takeoff_pose.pose.position.z = formation_height_;
        
        ros::Time takeoff_start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - takeoff_start < ros::Duration(15.0))) {
            setpoint_pub_.publish(takeoff_pose);
            
            // Check if takeoff is complete
            if (std::abs(current_pose_.pose.position.z - formation_height_) < 0.3) {
                takeoff_completed_ = true;
                ROS_INFO("%s: Despegue completado - Altura: %.2f", ns_.c_str(), 
                         current_pose_.pose.position.z);
                return true;
            }
            
            ros::spinOnce();
            rate.sleep();
        }
        
        ROS_WARN("%s: Timeout en despegue", ns_.c_str());
        return false;
    }
    
    void publishReady() {
        std_msgs::Bool ready_msg;
        ready_msg.data = true;
        ready_pub_.publish(ready_msg);
        ROS_INFO("%s: Publicando READY", ns_.c_str());
    }
    
    void land() {
        ROS_INFO("%s: Iniciando aterrizaje...", ns_.c_str());
        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = "AUTO.LAND";
        
        if (set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent) {
            ROS_INFO("%s: Modo aterrizaje activado", ns_.c_str());
        } else {
            // Fallback: gradual descent
            ros::Rate rate(20.0);
            geometry_msgs::PoseStamped land_pose = current_pose_;
            land_pose.pose.position.z = 0.0;
            
            for (int i = 0; i < 100 && ros::ok(); i++) {
                setpoint_pub_.publish(land_pose);
                ros::spinOnce();
                rate.sleep();
            }
        }
    }
    
    void updateConsensus() {
        if (neighbor_virtual_centers_.empty()) {
            // Sin vecinos: moverse hacia el centro deseado
            double dx = desired_center_.x - virtual_agent_center_.x;
            double dy = desired_center_.y - virtual_agent_center_.y;
            double dz = desired_center_.z - virtual_agent_center_.z;

            // Movimiento más lento hacia el centro deseado
            virtual_agent_center_.x += consensus_gain_ * dx;
            virtual_agent_center_.y += consensus_gain_ * dy;
            virtual_agent_center_.z += consensus_gain_ * dz;
        } else {
            // Con vecinos: consenso + atracción al centro deseado
            double avg_x = virtual_agent_center_.x;
            double avg_y = virtual_agent_center_.y;
            double avg_z = virtual_agent_center_.z;
            int count = 1;

            // Incluir vecinos en el promedio
            for (const auto& neighbor : neighbor_virtual_centers_) {
                avg_x += neighbor.second.x;
                avg_y += neighbor.second.y;
                avg_z += neighbor.second.z;
                count++;
            }

            avg_x /= count;
            avg_y /= count;
            avg_z /= count;

            // Consenso hacia el promedio
            double dx_consensus = avg_x - virtual_agent_center_.x;
            double dy_consensus = avg_y - virtual_agent_center_.y;
            double dz_consensus = avg_z - virtual_agent_center_.z;

            // Atracción hacia el centro deseado
            double dx_desired = desired_center_.x - virtual_agent_center_.x;
            double dy_desired = desired_center_.y - virtual_agent_center_.y;
            double dz_desired = desired_center_.z - virtual_agent_center_.z;

            // *** CRÍTICO: Reducir ganancias para evitar oscilaciones ***
            double consensus_weight = 0.7;  // Peso del consenso
            double desired_weight = 0.3;    // Peso del centro deseado

            virtual_agent_center_.x += consensus_gain_ * (consensus_weight * dx_consensus + desired_weight * dx_desired);
            virtual_agent_center_.y += consensus_gain_ * (consensus_weight * dy_consensus + desired_weight * dy_desired);
            virtual_agent_center_.z += consensus_gain_ * (consensus_weight * dz_consensus + desired_weight * dz_desired);

            ROS_DEBUG_THROTTLE(1.0, "%s: Consensus - Neighbors: %d, Avg center: (%.2f, %.2f), My center: (%.2f, %.2f)", 
                     ns_.c_str(), count-1, avg_x, avg_y, virtual_agent_center_.x, virtual_agent_center_.y);
        }

        calculateConsensusError();
        calculateFormationTrajectoryError();
        publishVirtualPose();
        publishFormationDebugInfo();
    }

    
    void publishVirtualPose() {
        // Publish virtual agent center for consensus (SIN offset de formación)
        geometry_msgs::PoseStamped virtual_center_msg;
        virtual_center_msg.pose.position = virtual_agent_center_;
        virtual_center_msg.header.stamp = ros::Time::now();
        virtual_center_msg.header.frame_id = "uav" + std::to_string(drone_id_);
        
        // Publish for consensus (esto es SOLO el centro, no la posición individual)
        virtual_pose_pub_.publish(virtual_center_msg);
        neighbor_pose_pub_.publish(virtual_center_msg);
        
        // *** SEPARACIÓN CLARA: La posición de formación individual se calcula SEPARADAMENTE ***
        // NO mezclar el centro virtual con la posición de formación aquí
        virtual_agent_pose_.pose.position.x = virtual_agent_center_.x;
        virtual_agent_pose_.pose.position.y = virtual_agent_center_.y;
        virtual_agent_pose_.pose.position.z = virtual_agent_center_.z;
        
        virtual_agent_pose_.header.stamp = ros::Time::now();
        virtual_agent_pose_.header.frame_id = "map";
        
        ROS_DEBUG("%s: Virtual center published: (%.2f, %.2f, %.2f)", 
                 ns_.c_str(), virtual_agent_center_.x, virtual_agent_center_.y, virtual_agent_center_.z);
    }

    
    // *** ADDED: Publish formation debug information ***
    void publishFormationDebugInfo() {
        std_msgs::Float64MultiArray debug_msg;
        debug_msg.data.clear();
        
        // Data structure: [drone_id, center_x, center_y, center_z, target_x, target_y, target_z, 
        //                  actual_distance, desired_radius, formation_error, consensus_error, trajectory_error]
        debug_msg.data.push_back(static_cast<double>(drone_id_));
        debug_msg.data.push_back(virtual_agent_center_.x);
        debug_msg.data.push_back(virtual_agent_center_.y);
        debug_msg.data.push_back(virtual_agent_center_.z);
        debug_msg.data.push_back(virtual_agent_pose_.pose.position.x);
        debug_msg.data.push_back(virtual_agent_pose_.pose.position.y);
        debug_msg.data.push_back(virtual_agent_pose_.pose.position.z);
        
        // Calculate actual distance from drone to center
        double dx = current_pose_.pose.position.x - virtual_agent_center_.x;
        double dy = current_pose_.pose.position.y - virtual_agent_center_.y;
        double actual_distance = sqrt(dx*dx + dy*dy);
        
        debug_msg.data.push_back(actual_distance);
        debug_msg.data.push_back(formation_radius_);
        debug_msg.data.push_back(current_formation_error_);
        debug_msg.data.push_back(current_consensus_error_);
        debug_msg.data.push_back(current_trajectory_error_);
        
        formation_debug_pub_.publish(debug_msg);
    }
    
    void publishPlannedTrajectory(const geometry_msgs::PoseStamped& planned_point) {
        planned_trajectory_pub_.publish(planned_point);
        
        // Also update internal state
        planned_trajectory_point_ = planned_point;
        planned_trajectory_received_ = true;
        
        ROS_DEBUG("%s: Published planned trajectory: (%.2f, %.2f, %.2f)", 
                 ns_.c_str(), planned_point.pose.position.x, planned_point.pose.position.y, planned_point.pose.position.z);
    }
    
    // *** MODIFIED: Enhanced formation position calculation with strict radius enforcement ***
    geometry_msgs::PoseStamped calculateFormationPosition() {
        geometry_msgs::PoseStamped target_pose;
        
        // Calcular la posición exacta deseada en el círculo
        double desired_x = virtual_agent_center_.x;
        double desired_y = virtual_agent_center_.y;
        double desired_z = virtual_agent_center_.z;
        
        // *** CRÍTICO: Añadir offset de formación aquí, NO en publishVirtualPose() ***
        if (drone_id_ >= 0 && drone_id_ < static_cast<int>(circle_offsets_.size())) {
            desired_x += circle_offsets_[drone_id_].first;
            desired_y += circle_offsets_[drone_id_].second;

            ROS_DEBUG_THROTTLE(1.0, "%s: Target position - Center: (%.2f, %.2f), Offset: (%.2f, %.2f), Final: (%.2f, %.2f)", 
                     ns_.c_str(), virtual_agent_center_.x, virtual_agent_center_.y,
                     circle_offsets_[drone_id_].first, circle_offsets_[drone_id_].second,
                     desired_x, desired_y);
        }

        // Control proporcional hacia la posición deseada
        double dx = desired_x - current_pose_.pose.position.x;
        double dy = desired_y - current_pose_.pose.position.y; 
        double dz = desired_z - current_pose_.pose.position.z;

        // Aplicar ganancia de control (más conservadora)
        target_pose.pose.position.x = current_pose_.pose.position.x + control_gain_ * dx;
        target_pose.pose.position.y = current_pose_.pose.position.y + control_gain_ * dy;
        target_pose.pose.position.z = current_pose_.pose.position.z + control_gain_ * dz;

        target_pose.header.stamp = ros::Time::now();
        target_pose.header.frame_id = "map";

        // Orientación hacia el centro del círculo (opcional)
        tf2::Quaternion q;
        double yaw = atan2(virtual_agent_center_.y - current_pose_.pose.position.y, 
                          virtual_agent_center_.x - current_pose_.pose.position.x);
        q.setRPY(0, 0, yaw);
        target_pose.pose.orientation = tf2::toMsg(q);
        
        // Debug crítico para verificar las posiciones
        double current_dist = sqrt(dx*dx + dy*dy);
        ROS_DEBUG_THROTTLE(2.0, "%s: Formation - Current dist to target: %.3f, Desired radius: %.3f, Error: %.3f", 
                 ns_.c_str(), current_dist, formation_radius_, 
                 std::abs(current_dist - formation_radius_));
        
        return target_pose;
    }
    
    // *** ADDED: Method to get formation statistics ***
    void getFormationStats(double& avg_distance, double& max_distance_error, int& neighbors_count) const {
        avg_distance = 0.0;
        max_distance_error = 0.0;
        neighbors_count = neighbor_virtual_centers_.size();
        
        // Calculate average distance from all drones to center
        double dx = current_pose_.pose.position.x - virtual_agent_center_.x;
        double dy = current_pose_.pose.position.y - virtual_agent_center_.y;
        double my_distance = sqrt(dx*dx + dy*dy);
        
        avg_distance = my_distance;
        max_distance_error = std::abs(my_distance - formation_radius_);
        
        // If we have neighbor information, we could calculate more comprehensive stats
        // For now, just return local drone stats
    }
    
    // *** ADDED: Method to check if formation is well-formed ***
    bool isFormationWellFormed(double tolerance = 0.5) const {
        double dx = current_pose_.pose.position.x - virtual_agent_center_.x;
        double dy = current_pose_.pose.position.y - virtual_agent_center_.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        return std::abs(distance - formation_radius_) < tolerance;
    }
    
    // Getters for monitoring
    double getConsensusError() const { return current_consensus_error_; }
    double getTrajectoryError() const { return current_trajectory_error_; }
    double getFormationError() const { return current_formation_error_; }  // *** ADDED: Formation error getter ***
    int getNeighborCount() const { return neighbor_virtual_centers_.size(); }
    bool isTakeoffCompleted() const { return takeoff_completed_; }
    bool isFormationReady() const { return formation_ready_; }
    bool isPlannedTrajectoryReceived() const { return planned_trajectory_received_; }
    double getFormationRadius() const { return formation_radius_; }  // *** ADDED: Formation radius getter ***
    int getTotalDrones() const { return total_drones_; }  // *** ADDED: Total drones getter ***
    
    // *** ADDED: Get current distance to formation center ***
    double getCurrentDistanceToCenter() const {
        double dx = current_pose_.pose.position.x - virtual_agent_center_.x;
        double dy = current_pose_.pose.position.y - virtual_agent_center_.y;
        return sqrt(dx*dx + dy*dy);
    }
    
    // *** ADDED: Get formation position for this drone ***
    std::pair<double, double> getFormationOffset() const {
        if (drone_id_ >= 0 && drone_id_ < static_cast<int>(circle_offsets_.size())) {
            return circle_offsets_[drone_id_];
        }
        return std::make_pair(0.0, 0.0);
    }
    
    // Setters for trajectory following
    void setDesiredCenter(const geometry_msgs::PoseStamped& center) {
        desired_center_ = center.pose.position;
        formation_ready_ = true;
        
        ROS_DEBUG("%s: Updated desired center: (%.2f, %.2f, %.2f)", 
                 ns_.c_str(), desired_center_.x, desired_center_.y, desired_center_.z);
    }
    
    // *** ADDED: Method to update formation parameters dynamically ***
    void updateFormationParameters(double radius, int total_drones = -1) {
        if (radius > 0) {
            formation_radius_ = radius;
            ROS_INFO("%s: Formation radius updated to %.2f", ns_.c_str(), formation_radius_);
        }
        
        if (total_drones > 0 && total_drones != total_drones_) {
            total_drones_ = total_drones;
            initializeCircleOffsets();  // Recalculate offsets
            ROS_INFO("%s: Total drones updated to %d", ns_.c_str(), total_drones_);
        }
    }
    
    // Legacy compatibility
    void setFormationCenter(const geometry_msgs::PoseStamped& center) {
        setDesiredCenter(center);
    }
    
    virtual void run() = 0;
};

#endif // BASE_NODE_H