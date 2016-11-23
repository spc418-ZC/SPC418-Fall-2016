#ifndef LASERSCAN_MAX_H
#define LASERSCAN_MAX_H

#include <publisher_subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <std_msgs/Float64.h>


template<>
void Publisher_Subscriber<std_msgs::Float64 , sensor_msgs::LaserScan>::subscriberCallback(const sensor_msgs::LaserScan::ConstPtr& receivedMsg)
{
    std_msgs::Float64 laser_max;
    float max = receivedMsg->intensities[0];
    for(int i=0; i < receivedMsg->intensities.size(); i++)
    {
        if(receivedMsg->ranges[i]>max)
        {
            max = receivedMsg->ranges[i];
        }
    }
    laser_max.data = (double) max;
    publisherObject.publish(laser_max);
}



#endif // LASERSCAN_MAX_H
