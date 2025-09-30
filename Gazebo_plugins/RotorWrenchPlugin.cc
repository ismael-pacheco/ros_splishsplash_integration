#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/WrenchStamped.h>

#include <boost/bind.hpp>
#include <thread>
#include <mutex>

namespace gazebo
{
  struct Rotor
  {
    std::string name;
    ignition::math::Vector3d position;
    geometry_msgs::WrenchStamped lastCmd;
    ros::Subscriber sub;
  };

  class RotorWrenchPlugin : public ModelPlugin
  {
  public:
    RotorWrenchPlugin() : ModelPlugin()
    {
    }

    ~RotorWrenchPlugin()
    {
      this->rosQueue.clear();
      this->rosQueue.disable();
      this->rosNode->shutdown();
      this->callbackQueueThread.join();
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;
      this->world = this->model->GetWorld();
      this->baseLink = this->model->GetLink("base_link");

      if (!this->baseLink)
      {
        gzerr << "[RotorWrenchPlugin] No se encontró el link base_link\n";
        return;
      }

      // Init ROS
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "rotor_wrench_plugin",
                  ros::init_options::NoSigintHandler);
      }
      
      this->rosNode.reset(new ros::NodeHandle("~"));

      // Usar una callback queue personalizada
      this->rosNode->setCallbackQueue(&this->rosQueue);

      // Definir posiciones de los rotores
      std::map<std::string, ignition::math::Vector3d> rotorPositions = {
        {"rotor_side_pos",      ignition::math::Vector3d(0, 0.4142148, -0.04029186)},
        {"rotor_side_neg",      ignition::math::Vector3d(0, -0.4142148, -0.04029186)},
        {"rotor_side_pos_top",  ignition::math::Vector3d(0, 0.4142148, 0.04029186)},
        {"rotor_side_neg_top",  ignition::math::Vector3d(0, -0.4142148, 0.04029186)},
        {"rotor_front_pos",     ignition::math::Vector3d(0.3312908, 0, -0.04029186)},
        {"rotor_front_neg",     ignition::math::Vector3d(0.3312908, 0, 0.04029186)},
        {"rotor_back_pos",      ignition::math::Vector3d(-0.3459592, 0, -0.04029186)},
        {"rotor_back_neg",      ignition::math::Vector3d(-0.3459592, 0, 0.04029186)}
      };

      for (auto &pair : rotorPositions)
      {
        Rotor r;
        r.name = pair.first;
        r.position = pair.second;

        std::string topicName = "/drone/" + r.name + "/wrench";
        r.sub = this->rosNode->subscribe<geometry_msgs::WrenchStamped>(
          topicName, 10,
          boost::bind(&RotorWrenchPlugin::OnRosMsg, this, _1, r.name));

        gzmsg << "[RotorWrenchPlugin] Suscrito a " << topicName 
              << " en posición (" << r.position.X() << ", " 
              << r.position.Y() << ", " << r.position.Z() << ")\n";

        this->rotors.push_back(r);
      }

      // Thread para procesar callbacks de ROS
      this->callbackQueueThread = std::thread(
        std::bind(&RotorWrenchPlugin::QueueThread, this));

      // Loop de actualización en cada paso de simulación
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RotorWrenchPlugin::OnUpdate, this));
      
      gzmsg << "[RotorWrenchPlugin] Plugin cargado con " << this->rotors.size() 
            << " rotores\n";
    }

    void OnRosMsg(const geometry_msgs::WrenchStampedConstPtr &msg, const std::string &name)
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      gzmsg << "[RotorWrenchPlugin] Recibido mensaje en " << name 
            << " - Fuerza: [" << msg->wrench.force.x << ", "
            << msg->wrench.force.y << ", " << msg->wrench.force.z << "]\n";
      
      for (auto &r : this->rotors)
      {
        if (r.name == name)
        {
          r.lastCmd = *msg;
          break;
        }
      }
    }

    void OnUpdate()
    {
      std::lock_guard<std::mutex> lock(this->mutex);

      if (!this->baseLink)
        return;

      static int counter = 0;
      bool anyForce = false;

      for (auto &r : this->rotors)
      {
        ignition::math::Vector3d force(
          r.lastCmd.wrench.force.x,
          r.lastCmd.wrench.force.y,
          r.lastCmd.wrench.force.z);

        ignition::math::Vector3d torque(
          r.lastCmd.wrench.torque.x,
          r.lastCmd.wrench.torque.y,
          r.lastCmd.wrench.torque.z);

        if (force.Length() > 0.1 || torque.Length() > 0.1)
          anyForce = true;

        // Aplicar fuerza al base_link en la posición del rotor
        this->baseLink->AddForceAtRelativePosition(force, r.position);
        this->baseLink->AddRelativeTorque(torque);
      }

      // Debug cada 100 iteraciones (~1 segundo con timestep normal)
      if (counter++ % 100 == 0 && anyForce)
      {
        auto vel = this->baseLink->WorldLinearVel();
        auto pos = this->baseLink->WorldPose().Pos();
        gzmsg << "[RotorWrenchPlugin] Pos: [" << pos.X() << ", " << pos.Y() 
              << ", " << pos.Z() << "] Vel: [" << vel.X() << ", " 
              << vel.Y() << ", " << vel.Z() << "]\n";
      }
    }

    void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  private:
    physics::ModelPtr model;
    physics::WorldPtr world;
    physics::LinkPtr baseLink;
    std::unique_ptr<ros::NodeHandle> rosNode;
    std::vector<Rotor> rotors;
    std::mutex mutex;
    event::ConnectionPtr updateConnection;
    
    // ROS callback queue y thread
    ros::CallbackQueue rosQueue;
    std::thread callbackQueueThread;
  };

  GZ_REGISTER_MODEL_PLUGIN(RotorWrenchPlugin)
}
