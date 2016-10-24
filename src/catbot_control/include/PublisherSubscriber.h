#ifndef PUBLISHER_SUBSCRIBER_H
#define PUBLISHER_SUBSCRIBER_H

#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>

template<typename PublishT,typename SubscribeT>
class PublisherSubscriber
{
public:
  PublisherSubscriber() {}
  PublisherSubscriber(std::string publishTopicName, std::string subscribeTopicName, int queueSize)
  {
    publisherObject  = nH.advertise<PublishT>(publishTopicName,queueSize);
    subscriberObject = nH.subscribe<SubscribeT>(subscribeTopicName,queueSize,&PublisherSubscriber::subscriberCallback,this);
  }
  void subscriberCallback(const typename SubscribeT::ConstPtr& recievedMsg);

protected:

  ros::Subscriber subscriberObject;
  ros::Publisher  publisherObject;
  ros::NodeHandle nH;
  tf::TransformBroadcaster tf_br;
};

#endif
