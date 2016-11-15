#ifndef PUBLISHER_SUBSCRIBER_H
#define PUBLISHER_SUBSCRIBER_H

#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>

template<typename PublishType, typename SubscribeType>

class Publisher_Subscriber
{
//The following 'public' lines can be used by any instance of this class and can be seen by rest of code
public:
    Publisher_Subscriber(std::string PublishTopic, std::string SubscribeTopic, int queueSize)
    {
        publisherObject = nHandle.advertise<PublishType>(PublishTopic,queueSize);

        subscriberObject = nHandle.subscribe<SubscribeType>(SubscribeTopic,queueSize,&Publisher_Subscriber::subscriberCallback, this);

    }

    void subscriberCallback(const typename SubscribeType::ConstPtr& receivedMsg);

//The following only belong to the specified instance of the class
private:
    ros::Subscriber subscriberObject;
    ros::Publisher publisherObject;
    ros::NodeHandle nHandle;
    tf::TransformBroadcaster tfBroad;
};

#endif // PUBLISHER_SUBSCRIBER_H
