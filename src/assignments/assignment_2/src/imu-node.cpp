#include <imu.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "imu_node");
  IMU imu_pos_estimator("imu_data","pose2D",1000);
  ros::Duration(3).sleep();
  ros::spin();
}
