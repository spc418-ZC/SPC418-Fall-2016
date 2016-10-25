#ifndef POSETOPATH_H
#define POSETOPATH_H
#include "PublisherSubscriber.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

template<>
PublisherSubscriber<nav_msgs::Path,geometry_msgs::PoseStamped>::PublisherSubscriber() {}

class PathGenerator : protected PublisherSubscriber<nav_msgs::Path,geometry_msgs::PoseStamped>
{
public:
  PathGenerator(std::string publishTopicName, std::string subscribeTopicName, int queueSize, std::string frame_name)
  {
    publisherObject  = nH.advertise<nav_msgs::Path>(publishTopicName,queueSize);
    subscriberObject = nH.subscribe<geometry_msgs::PoseStamped>(subscribeTopicName,queueSize,&PathGenerator::subscriberCallback,this);
    frame_id         = frame_name;
  }

  void subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr& receivedMsg)
  {
    geometry_msgs::PoseStamped poseMsg;

    poseMsg.pose            = receivedMsg->pose;
    poseMsg.header          = receivedMsg->header;

    const geometry_msgs::PoseStamped constPoseMsg = poseMsg;
    if (pathMsg.poses.size() < 5000)
    {
      pathMsg.poses.push_back(constPoseMsg);
    }
    else
    {
      pathMsg.poses.erase(pathMsg.poses.begin());
      pathMsg.poses.push_back(constPoseMsg);
    }

    pathMsg.header.frame_id = frame_id;
    publisherObject.publish(pathMsg);
  }
protected:
  nav_msgs::Path pathMsg;
  std::string frame_id;
};

#endif // POSETOPATH_H
