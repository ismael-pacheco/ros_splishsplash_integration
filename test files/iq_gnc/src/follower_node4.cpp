#include "base_node.h"

class FollowerNode : public BaseNode {
private:
    ros::Subscriber formation_center_sub_;
    ros::Subscriber leader_done_sub_;
    ros::Subscriber leader_ack_sub_;
    ros::Publisher individual_metrics_pub_;

    bool leader_done_;
    double ack_timeout_sec_;
    double formation_timeout_sec_;
    
    // *** ADDED: Better initialization tracking ***
    bool consensus_initialized_;
    ros::Time consensus_start_time_;

    void publishIndividualMetrics() {
        std_msgs::Float64MultiArray metrics_msg;
        metrics_msg.data.clear();
        
        // Formato del mensaje:
        // [drone_id, consensus_error, trajectory_error, timestamp]
        metrics_msg.data.push_back(static_cast<double>(drone_id_));
        metrics_msg.data.push_back(getConsensusError());
        metrics_msg.data.push_back(getTrajectoryError());
        metrics_msg.data.push_back(ros::Time::now().toSec());
        
        individual_metrics_pub_.publish(metrics_msg);
        
        // *** ADDED: Debug info for consensus monitoring ***
        ROS_INFO_THROTTLE(2.0, "%s: Consensus Error: %.3f, Trajectory Error: %.3f, Virtual Center: (%.2f, %.2f, %.2f), Desired: (%.2f, %.2f, %.2f)", 
                         ns_.c_str(), getConsensusError(), getTrajectoryError(),
                         virtual_agent_center_.x, virtual_agent_center_.y, virtual_agent_center_.z,
                         desired_center_.x, desired_center_.y, desired_center_.z);
    }

    void leaderAckCb(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) {
            ack_received_ = true;
            ROS_INFO("%s: ACK recibido del lider", ns_.c_str());
        }
    }

    void formationCenterCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // *** FIXED: Properly set formation center and track consensus initialization ***
        setFormationCenter(*msg);
        
        // *** ADDED: Initialize consensus tracking ***
        if (!consensus_initialized_) {
            consensus_initialized_ = true;
            consensus_start_time_ = ros::Time::now();
            ROS_INFO("%s: Consensus initialized. Initial virtual center: (%.2f, %.2f, %.2f), Target: (%.2f, %.2f, %.2f)", 
                     ns_.c_str(), 
                     virtual_agent_center_.x, virtual_agent_center_.y, virtual_agent_center_.z,
                     desired_center_.x, desired_center_.y, desired_center_.z);
        }
        
        ROS_DEBUG("%s: Formation center received: (%.2f, %.2f, %.2f)", 
                 ns_.c_str(), msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    void leaderDoneCb(const std_msgs::Bool::ConstPtr& msg) {
        leader_done_ = msg->data;
        if (leader_done_) {
            ROS_INFO("%s: Recibida señal DONE del lider", ns_.c_str());
        }
    }
    
    bool waitForAck(ros::Rate& rate) {
        ROS_INFO("%s: Esperando ACK del lider...", ns_.c_str());
        ros::Time start_time = ros::Time::now();
        
        while (ros::ok() && !ack_received_) {
            // Continue hovering while waiting for ACK
            setpoint_pub_.publish(current_pose_);
            
            // Republish ready signal periodically
            if (fmod((ros::Time::now() - start_time).toSec(), 2.0) < 0.05) {
                publishReady();
            }
            
            if ((ros::Time::now() - start_time).toSec() > ack_timeout_sec_) {
                ROS_ERROR("%s: Timeout esperando ACK del lider", ns_.c_str());
                return false;
            }
            
            ros::spinOnce();
            rate.sleep();
        }
        
        ROS_INFO("%s: ACK recibido, listo para formacion", ns_.c_str());
        return true;
    }
    
    bool waitForFormationCenter(ros::Rate& rate) {
        ROS_INFO("%s: Esperando centro de formacion...", ns_.c_str());
        ros::Time start_time = ros::Time::now();
        
        while (ros::ok() && !formation_ready_) {
            setpoint_pub_.publish(current_pose_);
            
            double elapsed = (ros::Time::now() - start_time).toSec();
            if (elapsed > formation_timeout_sec_) {
                ROS_WARN("%s: Timeout esperando centro de formacion", ns_.c_str());
                return false;
            }
            
            // Periodic status update
            ROS_INFO_THROTTLE(2.0, "%s: Esperando centro de formacion... (%.1fs elapsed)", 
                             ns_.c_str(), elapsed);
            
            ros::spinOnce();
            rate.sleep();
        }
        
        ROS_INFO("%s: Centro de formacion recibido", ns_.c_str());
        return true;
    }

    // *** ADDED: Wait for planned trajectory to ensure proper initialization ***
    bool waitForPlannedTrajectory(ros::Rate& rate, double timeout = 10.0) {
        ROS_INFO("%s: Esperando punto de trayectoria planificada...", ns_.c_str());
        ros::Time start_time = ros::Time::now();
        
        while (ros::ok() && !isPlannedTrajectoryReceived()) {
            setpoint_pub_.publish(current_pose_);
            
            double elapsed = (ros::Time::now() - start_time).toSec();
            if (elapsed > timeout) {
                ROS_WARN("%s: Timeout esperando trayectoria planificada, continuando...", ns_.c_str());
                return false;
            }
            
            ROS_INFO_THROTTLE(2.0, "%s: Esperando trayectoria planificada... (%.1fs elapsed)", 
                             ns_.c_str(), elapsed);
            
            ros::spinOnce();
            rate.sleep();
        }
        
        ROS_INFO("%s: Trayectoria planificada recibida", ns_.c_str());
        return true;
    }

public:
    FollowerNode(ros::NodeHandle& nh, const std::string& ns, int drone_id) : 
        BaseNode(nh, ns, drone_id), leader_done_(false), consensus_initialized_(false) {
        
        // Get parameters
        nh_.param<double>("ack_timeout_sec", ack_timeout_sec_, 30.0);
        nh_.param<double>("formation_timeout_sec", formation_timeout_sec_, 15.0);
        
        // Setup subscribers
        std::string ack_topic = ns + "/ack";
        leader_ack_sub_ = nh_.subscribe<std_msgs::Bool>(
            ack_topic, 10, &FollowerNode::leaderAckCb, this);
            
        formation_center_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/uav0/formation_center", 10, &FollowerNode::formationCenterCb, this);
            
        leader_done_sub_ = nh_.subscribe<std_msgs::Bool>(
            "/uav0/done", 10, &FollowerNode::leaderDoneCb, this);
        individual_metrics_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "/individual_metrics", 10);
            
        ROS_INFO("%s: Follower %d inicializado - Formacion circular", ns_.c_str(), drone_id);
        ROS_INFO("%s: Initial virtual center: (%.2f, %.2f, %.2f)", 
                 ns_.c_str(), virtual_agent_center_.x, virtual_agent_center_.y, virtual_agent_center_.z);
    }

    void run() override {
        ros::Rate rate(20.0);

        // Phase 1: Wait for FCU connection
        if (!waitForFCUConnection(rate)) {
            ROS_ERROR("%s: Fallo conexion FCU", ns_.c_str());
            return;
        }

        // Phase 2: Initialize setpoints
        initializeSetpoints(rate);
        
        // Phase 3: Set OFFBOARD mode
        if (!setOffboardMode(rate)) {
            ROS_ERROR("%s: Fallo modo OFFBOARD", ns_.c_str());
            return;
        }
        
        // Phase 4: Arm vehicle
        if (!armVehicle(rate)) {
            ROS_ERROR("%s: Fallo armado", ns_.c_str());
            return;
        }
        
        // Phase 5: Takeoff
        if (!takeoff(rate)) {
            ROS_ERROR("%s: Fallo despegue", ns_.c_str());
            return;
        }
        
        // Phase 6: Signal ready and wait for leader ACK
        publishReady();
        if (!waitForAck(rate)) {
            ROS_ERROR("%s: No se recibio ACK del lider, aterrizando", ns_.c_str());
            land();
            return;
        }
        
        // Phase 7: Wait for formation center
        if (!waitForFormationCenter(rate)) {
            ROS_ERROR("%s: No se recibio centro de formacion, aterrizando", ns_.c_str());
            land();
            return;
        }
        
        // *** ADDED: Phase 7.5: Wait for planned trajectory (optional with timeout) ***
        waitForPlannedTrajectory(rate, 5.0);  // 5 second timeout, non-critical
        
        // Phase 8: Formation flight with consensus algorithm
        ROS_INFO("%s: Iniciando vuelo en formacion circular con consenso!", ns_.c_str());
        ROS_INFO("%s: Initial consensus error should be visible now...", ns_.c_str());

        while (ros::ok()) {
            // Check for mission end signal
            if (leader_done_) {
                ROS_INFO("%s: Mision completada, aterrizando", ns_.c_str());
                land();
                return;
            }
        
            // *** FIXED: Update consensus to converge virtual centers ***
            updateConsensus();

            // *** ENHANCED: Publish individual metrics with both errors ***
            publishIndividualMetrics();

            // Calculate and publish formation position based on converged virtual center
            geometry_msgs::PoseStamped target_pose = calculateFormationPosition();
            setpoint_pub_.publish(target_pose);

            // *** ENHANCED: Debug information for both consensus and trajectory errors ***
            if (consensus_initialized_) {
                double consensus_time = (ros::Time::now() - consensus_start_time_).toSec();
                ROS_INFO_THROTTLE(3.0, "%s: t=%.1fs, Consensus Error: %.3f, Trajectory Error: %.3f, Neighbors: %d, Virtual: (%.2f,%.2f) -> Desired: (%.2f,%.2f)", 
                                 ns_.c_str(), consensus_time, getConsensusError(), getTrajectoryError(), getNeighborCount(),
                                 virtual_agent_center_.x, virtual_agent_center_.y,
                                 desired_center_.x, desired_center_.y);
                
                // *** ADDED: Log planned trajectory status ***
                if (isPlannedTrajectoryReceived()) {
                    ROS_DEBUG_THROTTLE(5.0, "%s: Planned trajectory active: (%.2f, %.2f, %.2f)", 
                                      ns_.c_str(), planned_trajectory_point_.pose.position.x, 
                                      planned_trajectory_point_.pose.position.y, 
                                      planned_trajectory_point_.pose.position.z);
                } else {
                    ROS_DEBUG_THROTTLE(5.0, "%s: No planned trajectory received yet", ns_.c_str());
                }
            }

            // Safety check
            if (!current_state_.armed || current_state_.mode != "OFFBOARD") {
                ROS_ERROR("%s: Estado critico perdido! Aterrizando...", ns_.c_str());
                land();
                return;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "follower_node");
    ros::NodeHandle nh;
    
    std::string ns;
    int drone_id;
    nh.param<std::string>("namespace", ns, "/uav4");
    nh.param<int>("drone_id", drone_id, 4);
    
    FollowerNode node(nh, ns, drone_id);
    node.run();
    
    return 0;
}