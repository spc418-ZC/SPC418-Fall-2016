#ifndef ODOMETRYTOPOSESTAMPED_H
#define ODOMETRYTOPOSESTAMPED_H

#include "PublisherSubscriber.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

double wheel_separation = 0.3159;
double wheel_diameter   = 0.10;

template<>
PublisherSubscriber<geometry_msgs::PoseStamped,nav_msgs::Odometry>::PublisherSubscriber() {}

class PoseGenerator : protected PublisherSubscriber<geometry_msgs::PoseStamped,nav_msgs::Odometry>
{
public:
  PoseGenerator(std::string publishTopicName, std::string subscribeTopicName, int queueSize, std::string frame_name)
  {
    publisherObject  = nH.advertise<geometry_msgs::PoseStamped>(publishTopicName,queueSize);
    subscriberObject = nH.subscribe<nav_msgs::Odometry>(subscribeTopicName,queueSize,&PoseGenerator::subscriberCallback,this);
    frame_id         = frame_name;
  }

  void subscriberCallback(const nav_msgs::Odometry::ConstPtr& recievedMsg)
  {
    geometry_msgs::PoseStamped poseMsg;

    poseMsg                 = recievedMsg->pose;
    poseMsg.header.frame_id = frame_id;

    publisherObject.publish(poseMsg);
  }
protected:
  std::string frame_id;
};
#endif // ODOMETRYTOPOSESTAMPED_H
