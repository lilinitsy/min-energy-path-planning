#include "Robot.h"


Robot::Robot()
{
	//pioneer_chassis = node_handle.advertise<gazebo_msgs::LinkState>("/gazebo/link_states", 1);
	client = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	modelstate.model_name = (std::string) "pioneer2dx";
}


void Robot::run()
{
	gazebo_msgs::LinkState chassis;
	//pioneer_chassis.publish(chassis);
	//ROS_INFO_STREAM(pioneer_chassis);
	geometry_msgs::Twist twist = geometry_msgs::Twist();
	float forward = 0.1f;
	twist.linear.x = forward;
	modelstate.twist = twist;
	modelstate.pose.position.x += 0.1f;

	gazebo_msgs::SetModelState setmodelstate;
	setmodelstate.request.model_state = modelstate;
	client.call(setmodelstate);
	ROS_INFO_STREAM(modelstate.twist);
}