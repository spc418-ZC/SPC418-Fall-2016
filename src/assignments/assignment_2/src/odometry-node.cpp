#include <joint2odom.h>

int main(int argc, char* argv[])
{
  ros::init(argc,argv, "odometry_node");
  Publisher_Subscriber<nav_msgs::Odometry , sensor_msgs::JointState> odometer("odometry", "joint_states", 1);
  ros::spin();
}
