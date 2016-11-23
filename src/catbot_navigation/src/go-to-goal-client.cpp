#include <ros/ros.h>
#include <catbot_navigation/GoToGoalAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <math.h>

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "gotogoalclient");
	actionlib::SimpleActionClient<catbot_navigation::GoToGoalAction> client("catbotGoToGoal",true);
	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	client.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

	catbot_navigation::GoToGoalGoal goalMsg;

	goalMsg.x_pos = -5;
	goalMsg.y_pos = -5;
	goalMsg.theta = M_PI;

	client.sendGoal(goalMsg);

	bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	 {
	   actionlib::SimpleClientGoalState state = client.getState();
	   ROS_INFO("Action finished: %s",state.toString().c_str());
	 }
	 else
	   ROS_INFO("Action did not finish before the time out.");

	 //exit
	 return 0;
}
