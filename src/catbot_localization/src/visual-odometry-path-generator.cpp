#include "PoseStampedToPath.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "odometry-pose-generator");
  PathGenerator pathgenerator("/catbot/vo_odometry_path","/stereo_odometer/pose",1,"world");
  ros::spin();
}
