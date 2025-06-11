#include <gnc_functions.hpp>
#include <cmath>

int main(int argc, char** argv)
{
    //initialize ros 
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");
    
    //initialize control publisher/subscribers
    init_publisher_subscriber(gnc_node);

    // wait for FCU connection
    wait4connect();

    initialize_local_frame();
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 3;

	ros::Rate rate(20.0); // 20 Hz
	for (int i = 0; i < 40; ++i) {
    	local_pos_pub.publish(pose);
   	 	ros::spinOnce();
   		rate.sleep();
	}

	//switch to mode GUIDED
	set_mode("OFFBOARD");
	arm();  // Usa la función armar definida en tu API, que llama a mavros_msgs::CommandBool
	//create local reference frame 


	//request takeoff
    //request takeoff
    takeoff(3);

    // Set a moderate speed for smooth movement
    set_speed(1.0); // 1 m/s

    // Parameters for circular trajectory
    const float radius = 5.0f;       // radius of the circle in meters
    const float center_x = 0.0f;     // center x coordinate
    const float center_y = 0.0f;     // center y coordinate
    const float base_z = 3.0f;       // base altitude
    const float z_amplitude = 1.0f;  // amplitude of z oscillation
    const float omega = 0.2f;        // angular velocity (rad/s)
    
    float angle = 0.0f;
    const float two_pi = 2.0f * M_PI;
    
    while(ros::ok())
    {
        ros::spinOnce();
        
        // Calculate next position on circular path
        float x = center_x + radius * cos(angle);
        float y = center_y + radius * sin(angle);
        
        // Calculate z position with sine wave oscillation
        float z = base_z + z_amplitude * sin(angle * 2.0f); // Double frequency for z
        
        // Calculate desired heading (tangent to the circle)
        float desired_heading = 0;
        
        // Set destination
        set_destination(x, y, z, desired_heading);
        
        // Increment angle for next iteration
        angle += omega * (1.0f/20.0f); // omega * dt
        if(angle > two_pi) angle -= two_pi;
        
        rate.sleep();
    }

    // Land when finished
    land();
    return 0;
}