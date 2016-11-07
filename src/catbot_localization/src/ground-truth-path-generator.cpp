#include "PoseStampedToPath.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ground-truth-path-generator");
  PathGenerator pathgenerator("ground_truth_path","ground_truth_pose",1,"world");
  ros::spin();
}
