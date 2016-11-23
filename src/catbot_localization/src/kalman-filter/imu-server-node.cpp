#include <ros/ros.h>
#include <imu.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_server_node");
	IMU imu_pos_estimator("imu_data","imuPose",10);
	ros::ServiceServer serverObject = imu_pos_estimator.nH.advertiseService("imuPose",&IMU::srvCallback,&imu_pos_estimator);
	ros::Duration(3).sleep();
	ros::spin();
}
