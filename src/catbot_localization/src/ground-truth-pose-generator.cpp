#include "ModelStatesToPoseStamped.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ground-truth-pose-generator");
  PoseGenerator posegenerator("/catbot/ground_truth_pose","/gazebo/model_states",1,"world");
  ros::spin();
}
