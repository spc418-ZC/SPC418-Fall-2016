#include "ground-truth.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_truth");
  PublisherSubscriber<nav_msgs::Odometry,gazebo_msgs::ModelStates> ground_truth("/catbot/ground_truth","/gazebo/model_states",1);
  ros::spin();
}
