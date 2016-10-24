#include "PoseStampedToPath.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "odometry-path-generator");
  PathGenerator pathgenerator("/catbot/odometry_path","/catbot/odometry_pose",1,"world");
  ros::spin();
}
