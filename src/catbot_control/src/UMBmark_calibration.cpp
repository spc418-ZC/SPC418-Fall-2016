#include <umbmark-calibration.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "UMBmark_calibration");

  UMBmarkMethod calibration("left_motor_controller/command","right_motor_controller/command", "ground_truth_pose", 1);
  ros::spin();
}
