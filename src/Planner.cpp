#include <iostream>


#include "Robot.h"

// https://answers.ros.org/question/12937/gazebo-service-through-c-code/
// MAY BE USEFUL
int main(int argc, char **argv)
{
	ros::init(argc, argv, "min-pe-planner");
	Robot robot = Robot();

	while(ros::ok())
	{
		robot.run();
		ros::spinOnce();
	}
	return 0;
}