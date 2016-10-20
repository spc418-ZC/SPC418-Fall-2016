#include "Odometer.h"

int main(int argc, char* argv[])
{
  ros::init(argc,argv,"encoder_node");

  PublisherSubscriber<nav_msgs::Odometry,sensor_msgs::JointState> encoder("/catbot/odom","/catbot/joint_states",1000);
  ros::spin();
}
