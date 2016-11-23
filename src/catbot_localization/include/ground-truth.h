#ifndef GROUNDTRUTH_H
#define GROUNDTRUTH_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include "PublisherSubscriber.h"
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <array>

template<>
void PublisherSubscriber<nav_msgs::Odometry,gazebo_msgs::ModelStates>::subscriberCallback(const gazebo_msgs::ModelStates::ConstPtr& receivedMsg)
{
  static unsigned int seq = 0;

  boost::array<double,36> cov = {0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0};

  size_t arrayLength = receivedMsg->name.size();

  geometry_msgs::Pose   poseMsg;
  geometry_msgs::Twist  twistMsg;

  tf::Quaternion qt;
  tf::Vector3 vt;

  for(size_t i = 0; i < arrayLength; ++i)
  {
    if (receivedMsg->name[i]=="catbot")
    {
      poseMsg  = receivedMsg->pose [i];
      twistMsg = receivedMsg->twist[i];
    }
  }

  qt.setW(poseMsg.orientation.w);
  qt.setX(poseMsg.orientation.x);
  qt.setY(poseMsg.orientation.y);
  qt.setZ(poseMsg.orientation.z);

  vt.setX(poseMsg.position.x);
  vt.setY(poseMsg.position.y);
  vt.setZ(poseMsg.position.z);

  const ros::Time current_time = ros::Time::now();
  const tf::Transform base_footprint_to_world( qt, vt );

  tf_br.sendTransform(tf::StampedTransform(base_footprint_to_world.inverse(),current_time,"base_footprint","world"));

  std_msgs::Header headerMsg;
  headerMsg.seq      = seq;
  headerMsg.stamp    = ros::Time::now();
  headerMsg.frame_id = "base_footprint";

  nav_msgs::Odometry odomMsg;

  odomMsg.pose.pose        = poseMsg;
  odomMsg.pose.covariance  = cov;
  odomMsg.twist.twist      = twistMsg;
  odomMsg.twist.covariance = cov;
  odomMsg.header           = headerMsg;
  odomMsg.child_frame_id   = "world";

  publisherObject.publish(odomMsg);
  seq++;
}

#endif // GROUNDTRUTH_H
