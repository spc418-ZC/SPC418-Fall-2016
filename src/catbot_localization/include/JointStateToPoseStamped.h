#ifndef JOINTSTATETOPOSESTAMPED_H
#define JOINTSTATETOPOSESTAMPED_H

#include "PublisherSubscriber.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>


double wheel_separation = 0.3159;
double wheel_diameter   = 0.10 * 0.763/0.87;

template<>
PublisherSubscriber<geometry_msgs::PoseStamped,sensor_msgs::JointState>::PublisherSubscriber() {}

class PoseGenerator : protected PublisherSubscriber<geometry_msgs::PoseStamped,sensor_msgs::JointState>
{
public:
  PoseGenerator(std::string publishTopicName, std::string subscribeTopicName, int queueSize, std::string frame_name, std::string right_wheel_name, std::string left_wheel_name)
  {
    publisherObject  = nH.advertise<geometry_msgs::PoseStamped>(publishTopicName,queueSize);
    subscriberObject = nH.subscribe<sensor_msgs::JointState>(subscribeTopicName,queueSize,&PoseGenerator::subscriberCallback,this);
    frame_id         = frame_name;
    left_wheel       = left_wheel_name;
    right_wheel      = right_wheel_name;
  }

  void subscriberCallback(const sensor_msgs::JointState::ConstPtr& recievedMsg)
  {
    geometry_msgs::PoseStamped poseMsg;
    double left_wheel_velocity = 0, right_wheel_velocity = 0;
    double dtheta = 0, dx = 0, dy = 0,dt;

    static double x = 0, y = 0, theta = 0;

    tf::Quaternion qt;

    for(size_t k = 0; k <  2; k++)
    {
      if (recievedMsg -> name[k] == left_wheel)
      {
        left_wheel_velocity  = recievedMsg -> velocity[k];
      }
      else if (recievedMsg -> name[k] == right_wheel)
      {
        right_wheel_velocity = recievedMsg -> velocity[k];
      }
    }

    dt      = 0.01;
    dtheta  = (right_wheel_velocity - left_wheel_velocity)*wheel_diameter * dt / wheel_separation;
    dx      = (right_wheel_velocity + left_wheel_velocity)*wheel_diameter * dt * cos(theta)/2;
    dy      = (right_wheel_velocity + left_wheel_velocity)*wheel_diameter * dt * sin(theta)/2;

    x     += dx;
    y     += dy;
    theta += dtheta;

    qt.setRPY(0,0,theta);

    poseMsg.pose.position.x    = x;
    poseMsg.pose.position.y    = y;
    poseMsg.pose.position.z    = 0;

    poseMsg.pose.orientation.w = qt.w();
    poseMsg.pose.orientation.x = qt.x();
    poseMsg.pose.orientation.y = qt.y();
    poseMsg.pose.orientation.z = qt.z();

    poseMsg.header.frame_id    = frame_id;
    poseMsg.header.stamp       = ros::Time::now();

    publisherObject.publish(poseMsg);
  }
protected:
  std::string frame_id;
  std::string left_wheel, right_wheel;
};

#endif // JOINTSTATETOPOSESTAMPED_H
