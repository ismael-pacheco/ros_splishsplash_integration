#include "base_node.h"
#include <map>
#include <cmath>

class LeaderNode : public BaseNode {
private:
    ros::Publisher formation_center_pub_;
    ros::Publisher done_pub_;
    ros::Publisher leader_trajectory_pub_;
    ros::Publisher formation_metrics_pub_;
    ros::Publisher individual_metrics_pub_;
    std::vector<ros::Publisher> ack_publishers_;
    std::vector<ros::Subscriber> follower_subs_;
    
    int followers_ready_count_;
    std::vector<bool> received_ready_;
    int num_followers_;
    double wait_timeout_sec_;
    
    // Formation trajectory variables
    ros::Time formation_start_time_;
    bool formation_active_;
    
    // Trajectory parameters
    double trajectory_radius_;
    double angular_velocity_;
    double vertical_amplitude_;
    double vertical_frequency_;
    
    // Formation monitoring
    std::vector<double> follower_distances_;
    double avg_formation_error_;
    ros::Time last_metrics_time_;
    
    // *** ADDED: Leader consensus tracking ***
    bool consensus_initialized_;
    ros::Time consensus_start_time_;

    void followerReadyCb(const std_msgs::Bool::ConstPtr& msg, int drone_id) {
        if (msg->data && !received_ready_[drone_id]) {
            ROS_INFO("%s: READY recibido de uav%d", ns_.c_str(), drone_id);
            received_ready_[drone_id] = true;
            followers_ready_count_++;
            
            // Send immediate ACK to specific drone
            std_msgs::Bool ack;
            ack.data = true;
            ack_publishers_[drone_id - 1].publish(ack);
            
            ROS_INFO("%s: ACK enviado a uav%d (%d/%d listos)", 
                    ns_.c_str(), drone_id, followers_ready_count_, num_followers_);
        }
    }

    bool waitForFollowersReady(ros::Rate& rate) {
        ROS_INFO("%s: Esperando que %d followers esten listos...", ns_.c_str(), num_followers_);
        ros::Time start_time = ros::Time::now();
        
        while (ros::ok() && followers_ready_count_ < num_followers_) {
            // Maintain hover position while waiting
            geometry_msgs::PoseStamped hover_pose = current_pose_;
            hover_pose.pose.position.z = formation_height_;
            setpoint_pub_.publish(hover_pose);
            
            if ((ros::Time::now() - start_time).toSec() > wait_timeout_sec_) {
                ROS_WARN("%s: Timeout esperando followers (%d/%d)", 
                        ns_.c_str(), followers_ready_count_, num_followers_);
                return false;
            }
            
            ROS_INFO_THROTTLE(2.0, "%s: En hover esperando followers (%d/%d)...", 
                            ns_.c_str(), followers_ready_count_, num_followers_);
            ros::spinOnce();
            rate.sleep();
        }
        
        ROS_INFO("%s: Todos los followers listos! Iniciando formacion...", ns_.c_str());
        return true;
    }
    
    geometry_msgs::PoseStamped calculateTrajectoryPoint(double t) {
        geometry_msgs::PoseStamped trajectory_point;
        
        // Circular trajectory with vertical oscillation
        trajectory_point.pose.position.x = trajectory_radius_ * cos(angular_velocity_ * t);
        trajectory_point.pose.position.y = trajectory_radius_ * sin(angular_velocity_ * t);
        trajectory_point.pose.position.z = formation_height_ + 
            vertical_amplitude_ * sin(vertical_frequency_ * angular_velocity_ * t);
        
        // Set orientation tangent to circle (facing forward in trajectory)
        tf2::Quaternion q;
        q.setRPY(0, 0, angular_velocity_ * t + M_PI/2);
        trajectory_point.pose.orientation = tf2::toMsg(q);
        
        trajectory_point.header.stamp = ros::Time::now();
        trajectory_point.header.frame_id = "map";
        
        return trajectory_point;
    }
    
    void executeFormationTrajectory() {
        if (!formation_active_) {
            formation_start_time_ = ros::Time::now();
            formation_active_ = true;
            consensus_initialized_ = true;
            consensus_start_time_ = ros::Time::now();
            ROS_INFO("%s: Iniciando trayectoria de formacion circular", ns_.c_str());
            ROS_INFO("%s: Leader initial virtual center: (%.2f, %.2f, %.2f)", 
                     ns_.c_str(), virtual_agent_center_.x, virtual_agent_center_.y, virtual_agent_center_.z);
        }
        
        double t = (ros::Time::now() - formation_start_time_).toSec();
        
        // Calculate trajectory point using mathematical expression
        formation_center_ = calculateTrajectoryPoint(t);
        
        // *** ADDED: Publish planned trajectory point for all nodes to receive ***
        publishPlannedTrajectory(formation_center_);
        
        // *** FIXED: Leader must also update its desired center from trajectory ***
        setDesiredCenter(formation_center_);
        
        // Publish trajectory point for monitoring
        leader_trajectory_pub_.publish(formation_center_);
        
        // Debug info
        ROS_DEBUG("%s: Trayectoria t=%.2f: pos(%.2f, %.2f, %.2f)", 
                 ns_.c_str(), t, 
                 formation_center_.pose.position.x,
                 formation_center_.pose.position.y,
                 formation_center_.pose.position.z);
    }
    
    void updateFormationMetrics() {
        // Calculate formation metrics for monitoring
        ros::Time now = ros::Time::now();
        if ((now - last_metrics_time_).toSec() < 0.1) return; // Update at 10Hz for better resolution
        
        last_metrics_time_ = now;
        
        // Calculate average consensus error from neighbors
        double total_consensus_error = getConsensusError();
        double total_trajectory_error = getTrajectoryError();
        int active_neighbors = getNeighborCount();
        
        // *** MODIFIED: Unified formation metrics array with consensus and trajectory errors ***
        std_msgs::Float64MultiArray metrics_msg;
        metrics_msg.data.clear();
        metrics_msg.data.push_back(total_consensus_error);      // Index 0: consensus error
        metrics_msg.data.push_back(total_trajectory_error);     // Index 1: trajectory error (formation center vs planned)
        metrics_msg.data.push_back(active_neighbors);           // Index 2: neighbor count
        metrics_msg.data.push_back(formation_radius_);          // Index 3: formation radius
        metrics_msg.data.push_back(now.toSec());                // Index 4: timestamp
        
        formation_metrics_pub_.publish(metrics_msg);
        
        // *** NUEVO: Publicar métricas individuales del líder ***
        std_msgs::Float64MultiArray individual_msg;
        individual_msg.data.clear();
        individual_msg.data.push_back(static_cast<double>(drone_id_)); // drone_id = 0 para líder
        individual_msg.data.push_back(total_consensus_error);
        individual_msg.data.push_back(total_trajectory_error);
        individual_msg.data.push_back(now.toSec());
        
        individual_metrics_pub_.publish(individual_msg);
        
        // *** ENHANCED: Better leader debugging ***
        if (consensus_initialized_) {
            double consensus_time = (now - consensus_start_time_).toSec();
            ROS_INFO_THROTTLE(3.0, "%s: LEADER t=%.1fs, Consensus: %.3f, Trajectory: %.3f, Neighbors: %d, Virtual: (%.2f,%.2f) -> Desired: (%.2f,%.2f)", 
                             ns_.c_str(), consensus_time, total_consensus_error, total_trajectory_error, active_neighbors,
                             virtual_agent_center_.x, virtual_agent_center_.y,
                             desired_center_.x, desired_center_.y);
        }
    }   

public:
    LeaderNode(ros::NodeHandle& nh, const std::string& ns) : 
        BaseNode(nh, ns, 0), followers_ready_count_(0), formation_active_(false),
        avg_formation_error_(0.0), consensus_initialized_(false) {
        
        // Get parameters
        nh_.param<int>("num_followers", num_followers_, 4);
        nh_.param<double>("wait_timeout_sec", wait_timeout_sec_, 30.0);
        
        // Trajectory parameters - smaller for visual verification
        nh_.param<double>("trajectory_radius", trajectory_radius_, 6.0);  // 6m radius
        nh_.param<double>("angular_velocity", angular_velocity_, 0.3);   // 0.3 rad/s
        nh_.param<double>("vertical_amplitude", vertical_amplitude_, 3); // 3m vertical oscillation
        nh_.param<double>("vertical_frequency", vertical_frequency_, 1); // 1x frequency for Z
        
        // Initialize tracking arrays
        received_ready_.resize(num_followers_ + 1, false);
        follower_distances_.resize(num_followers_, 0.0);
        last_metrics_time_ = ros::Time::now();
        
        // Setup publishers with consistent naming
        formation_center_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            ns_ + "/formation_center", 10);  // Now publishes to /uav0/formation_center
        leader_trajectory_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            ns_ + "/trajectory_point", 10);
        formation_metrics_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "/formation_metrics", 10);
        done_pub_ = nh_.advertise<std_msgs::Bool>(ns_ + "/done", 10);  // Consistent naming
        individual_metrics_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "/individual_metrics", 10);

        // Setup individual ACK publishers for each follower
        for (int i = 1; i <= num_followers_; i++) {
            std::string ack_topic = "/uav" + std::to_string(i) + "/ack";
            ack_publishers_.push_back(
                nh_.advertise<std_msgs::Bool>(ack_topic, 10, true) // latch = true
            );
        }

        // Subscribe to follower ready signals
        for (int i = 1; i <= num_followers_; i++) {
            std::string ready_topic = "/uav" + std::to_string(i) + "/ready";
            follower_subs_.push_back(
                nh_.subscribe<std_msgs::Bool>(
                    ready_topic, 10,
                    boost::bind(&LeaderNode::followerReadyCb, this, _1, i)
                )
            );
        }
        
        ROS_INFO("%s: Leader inicializado - Radio formacion: %.2fm, Radio trayectoria: %.2fm", 
                 ns_.c_str(), formation_radius_, trajectory_radius_);
        ROS_INFO("%s: Initial virtual center: (%.2f, %.2f, %.2f)", 
                 ns_.c_str(), virtual_agent_center_.x, virtual_agent_center_.y, virtual_agent_center_.z);
        ROS_INFO("%s: Esperando %d followers", ns_.c_str(), num_followers_);
    }

    void run() override {
        ros::Rate rate(20.0);

        // Phase 1: Wait for FCU connection
        ROS_INFO("%s: === FASE 1: Conexion FCU ===", ns_.c_str());
        if (!waitForFCUConnection(rate)) {
            ROS_ERROR("%s: Fallo conexion FCU", ns_.c_str());
            return;
        }

        // Phase 2: Initialize setpoints
        ROS_INFO("%s: === FASE 2: Inicializacion setpoints ===", ns_.c_str());
        initializeSetpoints(rate);
        
        // Phase 3: Set OFFBOARD mode
        ROS_INFO("%s: === FASE 3: Modo OFFBOARD ===", ns_.c_str());
        if (!setOffboardMode(rate)) {
            ROS_ERROR("%s: Fallo modo OFFBOARD", ns_.c_str());
            return;
        }
        
        // Phase 4: Arm vehicle
        ROS_INFO("%s: === FASE 4: Armado ===", ns_.c_str());
        if (!armVehicle(rate)) {
            ROS_ERROR("%s: Fallo armado", ns_.c_str());
            return;
        }
        
        // Phase 5: Takeoff
        ROS_INFO("%s: === FASE 5: Despegue ===", ns_.c_str());
        if (!takeoff(rate)) {
            ROS_ERROR("%s: Fallo despegue", ns_.c_str());
            return;
        }
        
        // Phase 6: Hover and wait for all followers to be ready
        ROS_INFO("%s: === FASE 6: Espera followers ===", ns_.c_str());
        if (!waitForFollowersReady(rate)) {
            ROS_ERROR("%s: No todos los followers listos, aterrizando", ns_.c_str());
            land();
            return;
        }
        
        // Phase 7: Brief stabilization period
        ROS_INFO("%s: === FASE 7: Estabilizacion ===", ns_.c_str());
        ros::Time stab_start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - stab_start).toSec() < 5.0) {
            geometry_msgs::PoseStamped stable_pose = current_pose_;
            stable_pose.pose.position.z = formation_height_;
            setpoint_pub_.publish(stable_pose);
            
            // Initialize formation center at current position  
            formation_center_ = stable_pose;
            formation_center_pub_.publish(formation_center_);
            
            // *** ADDED: Publish initial planned trajectory point during stabilization ***
            publishPlannedTrajectory(formation_center_);
            
            // *** ADDED: Early formation center broadcast ***
            ROS_INFO_THROTTLE(1.0, "%s: Broadcasting formation center: (%.2f, %.2f, %.2f)", 
                             ns_.c_str(), formation_center_.pose.position.x, 
                             formation_center_.pose.position.y, formation_center_.pose.position.z);
            
            ros::spinOnce();
            rate.sleep();
        }

        // Phase 8: Formation flight
        ROS_INFO("%s: === FASE 8: Vuelo en formacion ===", ns_.c_str());
        ROS_INFO("%s: Iniciando trayectoria circular - Radio: %.1fm, Vel: %.3f rad/s", 
                 ns_.c_str(), trajectory_radius_, angular_velocity_);
        
        ros::Time mission_start = ros::Time::now();
        const double mission_duration = 90.0; // 90 seconds for demonstration
        
        while (ros::ok() && (ros::Time::now() - mission_start).toSec() < mission_duration) {
            // *** FIXED: Execute mathematical trajectory (leader follows trajectory too) ***
            executeFormationTrajectory();
            
            // *** FIXED: Leader also participates in consensus algorithm ***
            updateConsensus();
            
            // Publish formation center for followers - HIGH FREQUENCY
            formation_center_pub_.publish(formation_center_);
            
            // *** FIXED: Calculate and publish leader's position in formation ***
            geometry_msgs::PoseStamped target_pose = calculateFormationPosition();
            setpoint_pub_.publish(target_pose);
            
            // Update and publish formation metrics
            updateFormationMetrics();
            
            // Safety check
            if (!current_state_.armed || current_state_.mode != "OFFBOARD") {
                ROS_ERROR("%s: Estado critico perdido! Aterrizando...", ns_.c_str());
                break;
            }
            
            // Progress report
            double elapsed = (ros::Time::now() - mission_start).toSec();
            ROS_INFO_THROTTLE(10.0, "%s: Progreso mision: %.1f/%.1f segundos (%.1f%%)", 
                             ns_.c_str(), elapsed, mission_duration, 
                             (elapsed/mission_duration)*100.0);
            
            ros::spinOnce();
            rate.sleep();
        }
        
        // Phase 9: Mission complete - signal followers and land
        ROS_INFO("%s: === FASE 9: Finalizacion ===", ns_.c_str());
        ROS_INFO("%s: Mision completada, enviando señal DONE", ns_.c_str());
        std_msgs::Bool done_msg;
        done_msg.data = true;
        
        // Send DONE signal multiple times to ensure delivery
        for (int i = 0; i < 30; i++) {
            done_pub_.publish(done_msg);
            
            // Continue publishing setpoint during shutdown signal
            setpoint_pub_.publish(current_pose_);
            
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        
        // Final hover before landing
        ROS_INFO("%s: Hover final antes de aterrizaje...", ns_.c_str());
        ros::Time final_hover = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - final_hover).toSec() < 3.0) {
            setpoint_pub_.publish(current_pose_);
            ros::spinOnce();
            rate.sleep();
        }
        
        land();
        ROS_INFO("%s: Mision del lider completada", ns_.c_str());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "leader_node");
    ros::NodeHandle nh;
    
    ROS_INFO("=== INICIANDO NODO LIDER DE FORMACION ===");
    
    LeaderNode node(nh, "/uav0");
    node.run();
    
    ROS_INFO("=== FINALIZANDO NODO LIDER ===");
    return 0;
}