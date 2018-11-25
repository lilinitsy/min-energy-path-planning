#include "Robot.h"


Robot::Robot()
{
	pioneer_chassis = node_handle.advertise<gazebo_msgs::LinkState>("/gazebo/link_states", 1);
}


void Robot::run()
{
	gazebo_msgs::LinkState chassis;
	pioneer_chassis.publish(chassis);
	ROS_INFO_STREAM(pioneer_chassis);
}