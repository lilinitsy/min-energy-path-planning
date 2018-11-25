#ifndef ROBOT_H
#define ROBOT_H

#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/ModelState.h>



/*
/gazebo publishes	/gazebo/link_states
					/gazebo/model_states
					/gazebo/parameter_descriptions
					/gazebo/parameter_updates

/gazebo subscribes	/gazebo/set_model_state
					/gazebo/set_link_state
*/



class Robot
{
	public:
		ros::NodeHandle node_handle;

		// these have to all publish poses...

		// name: pioneer2dx::chassis
		ros::Publisher pioneer_chassis;
		// name: pioneer2dx::right_wheel
		ros::Publisher pioneer_right_wheel;
		// name: pioneer2dx::left_wheel
		ros::Publisher pioneer_left_wheel;

		Robot();
		void run();
};


#endif