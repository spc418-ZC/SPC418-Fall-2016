#include <GotoGoalController.h>
#include <tf/transform_datatypes.h>
#include <iostream>

using std::cout;
using std::endl;

GotoGoalController::GotoGoalController(std::string name, std::string commandVelTopicName, std::string odometryTopicName)
	: controller(nH, name, boost::bind(&GotoGoalController::executeCallback, this, _1), false), actionServerName(name)
{
	controller.start();
	publisherObject  = nH.advertise<geometry_msgs::Twist>(commandVelTopicName,10);
	subscriberObject = nH.subscribe<nav_msgs::Odometry>(odometryTopicName,1,&GotoGoalController::subscriberCallback, this);
}

GotoGoalController::~GotoGoalController(void) {}

void GotoGoalController::executeCallback(const catbot_navigation::GoToGoalGoal::ConstPtr &receivedGoal)
{
	ros::Rate loop_rate(50); // This is a rate object used to control the rate of execution of the feedback loop
	bool success = false; // A flag to indicate whether the goal has been accomlplished or not

	ROS_INFO("Executing, Target co-ordinates are: \t x_pos: %3.3f, y_pos: %3.3f,  theta: %1.5f ...." ,receivedGoal->x_pos, receivedGoal->y_pos, receivedGoal->theta); // For debugging

	// Tolerance values used to indicate that the goal has been reached to avoid controller instability in the vicinity of the goal
	double eps_x = 0.1*receivedGoal->x_pos;
	double eps_Y = 0.1*receivedGoal->y_pos;
	double eps_t = 0.02*receivedGoal->theta;

	// Variables used to store the error in the states of the system
	double err_X      = 0.02, err_Y     = 0.02, err_T    = 0.02;
	// Variables used to store the previous values of the transformed states of the system
	double prev_alpha = 0.00, prev_beta = 0.00, prev_rho = 0.00;

	// Variabls used to store the transformed states of the system and their derivatives
	double  alpha,  beta,  rho;
	double dalpha, dbeta, drho;

	// Variables used to store the calculated controller output
	double v,w;

	while(!success)
	{
		if (controller.isPreemptRequested() || !ros::ok()) // Check whether a preempting request has been issued by the client OR Ctrl+C kill sequence has been issued.
		{
			ROS_INFO("%s: Preempted", actionServerName.c_str());
			controller.setPreempted(); // raise the preemption flag
			success = false; // Set the value of the success flag to zero indicating a failure
			break;
		}
		ros::spinOnce();

		/* Calculating the errors in the cartesian co-ordinates */
		err_X	= receivedGoal->x_pos - x_pos;
		err_Y	= receivedGoal->y_pos - y_pos;
		err_T	= receivedGoal->theta - theta;

		/* Calculating the associated error in the cylindrical co-ordinates*/
		rho		= sqrt(err_X*err_X + err_Y*err_Y);
		alpha	= -theta + atan2(err_Y, err_X);
		beta	= -theta - alpha;

		/* Calculating the derivatives of the error expressed in cylindrical co-ordinates */
		drho	= rho	- prev_rho;
		dbeta	= beta	- prev_beta;
		dalpha	= alpha - prev_alpha;

		/* Calculating controller output from the error and controller gains
			The input of the controller is the errors expressed in cylindrical co-ordinates and their associated derivatives
			The control signal is calculated using the control law given in R. Siegwart's book: "Introduction to Autonomous Mobile Robots", chapter 3
			The given control law is modified and morphed into a PD controller of the same archeitecture.
		*/
		v		= ((alpha < M_PI/2) && (alpha > -M_PI/2))? (kp_p * rho + kd_p * drho) :
														   -(kp_p * rho + kd_p * drho);
		w		= (kp_a * alpha + kd_a * dalpha)+ (kp_b * beta + kd_a * dbeta);

		success = (fabs(err_X) < eps_x) && (fabs(err_Y) < eps_Y); // Checking whether the goal has been reached or not

		if (success) // If the goal is reached, clean and end the processing, and report the successful execution of the request.
		{
				resultMsg.targetReached = success;
				ROS_INFO("%s: Succeeded", actionServerName.c_str());
				controller.setSucceeded(resultMsg);
				break;
		}

		// Fill in the fields for the feedback message and publish it
		feedbackMsg.err_x = err_X;
		feedbackMsg.err_y = err_Y;
		feedbackMsg.e_psi = err_T;

		controller.publishFeedback(feedbackMsg);

		// Debugging
		cout << " Goal Co-Ordinates are: " << receivedGoal->x_pos << " , " << receivedGoal->y_pos << endl;
		cout << " Current Location is  : " << x_pos << " , " << y_pos << endl;
		cout << " Current Attitude is  : " << theta << endl;
		cout << " Request Attitude is  : " << receivedGoal->theta << endl;
		cout << endl;
		cout << " ================= " << endl;
		cout << endl;
		cout << " Calculated control   Signals are,  V = " << v << " and W = " << w << endl;
		cout << " Error in Cartesian   Co-ordinates are: " << err_X << " , " << err_Y << " , " << err_T << endl;
		cout << " Error in Cylindrical Co-ordinates are: " << rho << " , " << alpha << " , " << beta << endl;
		cout << endl;
		cout << endl;


		// Publish the control signal
		geometry_msgs::Twist velocityCommand;

		velocityCommand.linear.x  = v;
		velocityCommand.linear.y  = 0;
		velocityCommand.linear.z  = 0;

		velocityCommand.angular.x = 0;
		velocityCommand.angular.y = 0;
		velocityCommand.angular.z = w;

		publisherObject.publish(velocityCommand);

		prev_rho	= rho;
		prev_alpha	= alpha;
		prev_beta	= beta;
		loop_rate.sleep();
	}

}

void GotoGoalController::subscriberCallback(const nav_msgs::Odometry::ConstPtr& receivedMsg)
{
	x_pos = receivedMsg->pose.pose.position.x;
	y_pos = receivedMsg->pose.pose.position.y;
	theta = tf::getYaw(receivedMsg->pose.pose.orientation);
}
