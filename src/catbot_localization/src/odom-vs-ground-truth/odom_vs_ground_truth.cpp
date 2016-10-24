#include <ros/ros.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "odom_vs_ground_truth");

  ROS_INFO("Hello world!");
}
