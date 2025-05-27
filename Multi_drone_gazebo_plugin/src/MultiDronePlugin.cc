#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <map>

namespace gazebo {

class MultiDronePlugin : public ModelPlugin {
public:
  MultiDronePlugin() {}

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = nullptr;
      ros::init(argc, argv, "multi_drone_plugin", ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("multi_drone_plugin"));
    this->world = _model->GetWorld();

    if (_sdf->HasElement("drones")) {
      sdf::ElementPtr droneElem = _sdf->GetElement("drones");
      while (droneElem) {
        std::string name = droneElem->Get<std::string>("name");
        physics::ModelPtr droneModel = world->ModelByName(name);
        if (droneModel) {
          this->drones[name] = droneModel;
          this->desiredPositions[name] = droneModel->WorldPose().Pos();

          ros::Subscriber sub = this->rosNode->subscribe<geometry_msgs::PoseStamped>(
            "/" + name + "/desired_pose", 1,
            boost::bind(&MultiDronePlugin::OnPoseMsg, this, _1, name));

          subscribers[name] = sub;
        }
        droneElem = droneElem->GetNextElement("drones");
      }
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MultiDronePlugin::OnUpdate, this));

    this->rosQueueThread = std::thread(std::bind(&MultiDronePlugin::QueueThread, this));
  }

  void OnPoseMsg(const geometry_msgs::PoseStamped::ConstPtr &msg, const std::string &name) {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->desiredPositions[name] = ignition::math::Vector3d(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);
  }

  void OnUpdate() {
    std::lock_guard<std::mutex> lock(this->mutex);
    for (auto& [name, drone] : this->drones) {
      auto currentPose = drone->WorldPose();
      ignition::math::Vector3d currentPos = currentPose.Pos();
      ignition::math::Vector3d desiredPos = this->desiredPositions[name];

      ignition::math::Vector3d error = desiredPos - currentPos;

      // Control de posición
      ignition::math::Vector3d cmdAngles;
      double k_p_pos = 2.0;
      cmdAngles.X() = k_p_pos * error.Y(); // pitch
      cmdAngles.Y() = -k_p_pos * error.X(); // roll

      // Altitud
      double thrust = baseThrust + k_p_z * error.Z();

      // Control de orientación
      ignition::math::Vector3d currentRot = currentPose.Rot().Euler();
      ignition::math::Vector3d torque;
      double k_p_att = 1.0;
      torque.X() = k_p_att * (cmdAngles.X() - currentRot.X());
      torque.Y() = k_p_att * (cmdAngles.Y() - currentRot.Y());

      // Aplicar fuerza/torque
      drone->GetLink("base_link")->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));
      drone->GetLink("base_link")->AddRelativeTorque(torque);
    }
  }

private:
  void QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  physics::WorldPtr world;
  std::unordered_map<std::string, physics::ModelPtr> drones;
  std::unordered_map<std::string, ignition::math::Vector3d> desiredPositions;

  std::unique_ptr<ros::NodeHandle> rosNode;
  std::map<std::string, ros::Subscriber> subscribers;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  std::mutex mutex;

  event::ConnectionPtr updateConnection;

  // Parámetros
  double baseThrust = 9.8;
  double k_p_z = 15.0;
};

GZ_REGISTER_MODEL_PLUGIN(MultiDronePlugin)
}  // namespace gazebo

