#include <ros/ros.h>
#include <GotoGoalController.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gotogoal");
	GotoGoalController Controller(ros::this_node::getName(),"velocity_command", "odometry");
	ros::spin();
}
