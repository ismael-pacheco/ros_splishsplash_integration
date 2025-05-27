#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/drone1/desired_pose", 10);

  ros::Rate rate(10);
  while (ros::ok()) {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = 1.0;
    msg.pose.position.y = 1.0;
    msg.pose.position.z = 2.0;

    pub.publish(msg);
    rate.sleep();
  }

  return 0;
}

