#include "JointStateToPoseStamped.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "odometry-pose-generator");
  PoseGenerator posegenerator("/catbot/odometry_pose","/catbot/joint_states",1,"world");
  ros::spin();
}
