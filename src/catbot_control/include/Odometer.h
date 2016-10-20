#ifndef PUBLISHER_SUBSCRIBER_ODOMETER_H
#define PUBLISHER_SUBSCRIBER_ODOMETER_H

#include "PublisherSubscriber.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>

#include <math.h>

double wheel_separation = 0.16*2;

template <>
void PublisherSubscriber<nav_msgs::Odometry,sensor_msgs::JointState>::subscriberCallback(const sensor_msgs::JointState::ConstPtr& recievedMsg)
{
  double left_wheel_position = 0, left_wheel_velocity = 0, right_wheel_position = 0, right_wheel_velocity = 0;
  double theta = 0, dx = 0, dy = 0, w = 0, dt = 0, x = 0, y = 0;
  double dt_secs = 0, dt_nsecs = 0;
  static double secs = 0, nsecs = 0;

  dt_secs  = recievedMsg->header.stamp.sec  -  secs;
  dt_nsecs = recievedMsg->header.stamp.nsec - nsecs;

  secs     = recievedMsg->header.stamp.sec;
  nsecs    = recievedMsg->header.stamp.nsec;

  ROS_INFO("dt_secs is: %f, and dt_nsecs is: %f",dt_secs,dt_nsecs);
  dt = (double) dt_secs + 0.000000001 * (double) dt_nsecs;
  tf::TransformBroadcaster tf_br;
  tf::Quaternion qt;
  tf::Vector3 vt;

  for(size_t k = 0; k <  2 /*SizeOfArray<const _name_type&>(recievedMsg->name)*/; k++)
  {
    if (recievedMsg -> name[k] == "left_motor")
    {
      left_wheel_position  = recievedMsg -> position[k];
      left_wheel_velocity  = recievedMsg -> velocity[k];
    }
    else if (recievedMsg -> name[k] == "right_motor")
    {
      right_wheel_position = recievedMsg -> position[k];
      right_wheel_velocity = recievedMsg -> velocity[k];
    }
  }

  theta = (right_wheel_position - left_wheel_position)*wheel_separation;
  w     = (right_wheel_velocity - left_wheel_velocity)*wheel_separation;

  x    = (right_wheel_position + left_wheel_position)/2 * cos(theta + w * dt);
  y    = (right_wheel_position + left_wheel_position)/2 * sin(theta + w * dt);
  dx   = (right_wheel_velocity + left_wheel_velocity)/2 * cos(theta);
  dy   = (right_wheel_velocity + left_wheel_velocity)/2 * sin(theta);

  qt.setRPY(0,0,theta);
  vt = tf::Vector3(x,y,0);
  const ros::Time current_time = ros::Time::now();
  const tf::Transform base_footprint_to_odom ( qt, vt );
  tf_br.sendTransform(tf::StampedTransform(base_footprint_to_odom,current_time,"base_footprint","odom"));
  nav_msgs::Odometry odomMsg;
  odomMsg.pose.pose.position.x = vt.x();
  odomMsg.pose.pose.position.y = vt.y();
  odomMsg.pose.pose.position.z = vt.z();

  odomMsg.pose.pose.orientation.x = qt.x();
  odomMsg.pose.pose.orientation.y = qt.y();
  odomMsg.pose.pose.orientation.z = qt.z();
  odomMsg.pose.pose.orientation.w = qt.w();

  odomMsg.twist.twist.angular.x   = 0.0;
  odomMsg.twist.twist.angular.y   = 0.0;
  odomMsg.twist.twist.angular.z   =  w ;

  odomMsg.twist.twist.linear.x    = dx;
  odomMsg.twist.twist.linear.y    = dy;
  odomMsg.twist.twist.linear.z    = 0.0;

  odomMsg.pose.covariance[0]       = 0.0001;
  odomMsg.pose.covariance[7]       = 0.0001;
  odomMsg.pose.covariance[14]      = 100000000.0;
  odomMsg.pose.covariance[21]      = 100000000.0;
  odomMsg.pose.covariance[28]      = 100000000.0;
  odomMsg.pose.covariance[35]      = 0.0001;

  std_msgs::Header headerMsg;

  headerMsg = recievedMsg->header;
  headerMsg.frame_id = "base_footprint";

  odomMsg.header = headerMsg;
  odomMsg.child_frame_id = "odom";

  publisherObject.publish(odomMsg);
}

#endif
