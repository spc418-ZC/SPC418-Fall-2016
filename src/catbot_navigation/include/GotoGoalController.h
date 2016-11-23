#ifndef GOTOGOALCONTROLLER_H
#define GOTOGOALCONTROLLER_H

#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <catbot_navigation/GoToGoalAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class GotoGoalController
{
	public:
		GotoGoalController(std::string name, std::string commandVelTopicName, std::string odometryTopicName);
		~GotoGoalController(void);
		void executeCallback(const catbot_navigation::GoToGoalGoal::ConstPtr& receivedGoal);
		void subscriberCallback(const nav_msgs::Odometry::ConstPtr& receivedMsg);

	protected:
		ros::NodeHandle nH;
		ros::Publisher publisherObject;
		ros::Subscriber subscriberObject;

		actionlib::SimpleActionServer<catbot_navigation::GoToGoalAction> controller;
		catbot_navigation::GoToGoalFeedback feedbackMsg;
		catbot_navigation::GoToGoalResult resultMsg;
		std::string actionServerName;
		double x_pos = 0, y_pos = 0, theta = 0;
		double kp_p = M_PI/2, kp_a = 9/2, kp_b = -4/5;
		double kd_p = M_PI/3, kd_a = 9/3, kd_b = -4/6;

};

#endif // GOTOGOALCONTROLLER_H
