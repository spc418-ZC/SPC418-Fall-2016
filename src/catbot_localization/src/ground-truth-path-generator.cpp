#include "PoseStampedToPath.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ground-truth-path-generator");
  PathGenerator pathgenerator("/catbot/ground_truth_path","/catbot/ground_truth_pose",1,"world");
  ros::spin();
}
