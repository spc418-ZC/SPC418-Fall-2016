#include "JointStateToPoseStamped.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "odometry-pose-generator");
  PoseGenerator posegenerator("odometry_pose","joint_states",1,"world","right_motor", "left_motor");
  ros::spin();
}
