#include <ros/ros.h>
#include <kalman-filter/kalman-filter.h>

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "kalman_filter_node");
	KalmanFilter kalmanFilter;
	kalmanFilter.initializeFilter("refined_states","joint_states","imuPose");
	ros::spin();
}
