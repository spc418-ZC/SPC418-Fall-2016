#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "umbmark-simple");
  ros::NodeHandle nH;
  ros::Publisher rightMotorPub = nH.advertise<std_msgs::Float64>("right_motor_controller/command",1);
  ros::Publisher leftMotorPub  = nH.advertise<std_msgs::Float64>("left_motor_controller/command",1);
  std_msgs::Float64 rmCS, lmCS;
  rmCS.data = 0;  lmCS.data = 0;
  double square_size = 3, sample_time = 0.025;
  std::string current_stage = "ready";
  ros::Rate loop_rate(1/sample_time);

  double t = 0;
  while(ros::ok())
  {
    t += sample_time;

    if (current_stage == "ready")
    {
      rmCS.data = square_size;
      lmCS.data = square_size;
      current_stage = "executing-one";
    }

    if ((current_stage == "executing-one") && t >=3)
    {
      current_stage = "stage-two-ready";
      rmCS.data = 0;
      lmCS.data = 0;
    }



    rightMotorPub.publish(rmCS);
    leftMotorPub.publish(lmCS);

  }

  ROS_INFO("Hello world!");
}
