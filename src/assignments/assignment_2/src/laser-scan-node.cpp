#include <laserscan_max.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "laser_scan_node");
  Publisher_Subscriber<std_msgs::Float64 , sensor_msgs::LaserScan> laserAnalysis("laser_data_max", "laser/scan" , 1);
  ros::spin();
}
