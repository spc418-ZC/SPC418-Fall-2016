#include <ros/ros.h>
#include <PublisherSubscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

PublisherSubscriber<std_msgs::Float64, geometry_msgs::PoseStamped>::PublisherSubscriber(){ }

PublisherSubscriber<std_msgs::Float64, geometry_msgs::PoseStamped>::subscriberCallback(const geometry_msgs::PoseStamped_::ConstPtr &recievedMsg)
{

}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "UMBmark_calibration");

  ROS_INFO("Hello world!");
}
