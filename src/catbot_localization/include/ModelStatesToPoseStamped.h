#ifndef MODELSTATETOPOSESTAMPED_H
#define MODELSTATETOPOSESTAMPED_H

#include "PublisherSubscriber.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>

template<>
PublisherSubscriber<geometry_msgs::PoseStamped,gazebo_msgs::ModelStates>::PublisherSubscriber() {}

class PoseGenerator : protected PublisherSubscriber<geometry_msgs::PoseStamped,gazebo_msgs::ModelStates>
{
public:
  PoseGenerator(std::string publishTopicName, std::string subscribeTopicName, int queueSize, std::string frame_name)
  {
    publisherObject  = nH.advertise<geometry_msgs::PoseStamped>(publishTopicName,queueSize);
    subscriberObject = nH.subscribe<gazebo_msgs::ModelStates>(subscribeTopicName,queueSize,&PoseGenerator::subscriberCallback,this);
    frame_id         = frame_name;
  }

  void subscriberCallback(const gazebo_msgs::ModelStates::ConstPtr& receivedMsg)
  {
    geometry_msgs::PoseStamped poseMsg;

    for(size_t i = 0; i < receivedMsg->name.size(); ++i)
    {
      if (receivedMsg->name[i]=="catbot")
      {
        poseMsg.pose  = receivedMsg->pose [i];
      }
    }

    poseMsg.header.frame_id    = frame_id;
    poseMsg.header.stamp       = ros::Time::now();

    publisherObject.publish(poseMsg);
  }
protected:
  std::string frame_id;
};


#endif // MODELSTATETOPOSESTAMPED_H
