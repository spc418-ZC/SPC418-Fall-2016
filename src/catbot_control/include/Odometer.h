#ifndef PUBLISHER_SUBSCRIBER_ODOMETER_H
#define PUBLISHER_SUBSCRIBER_ODOMETER_H

#include "PublisherSubscriber.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <math.h>

double wheel_separation = 0.3159;
double wheel_diameter   = 0.10;

template <>
void PublisherSubscriber<nav_msgs::Odometry,sensor_msgs::JointState>::subscriberCallback(const sensor_msgs::JointState::ConstPtr& recievedMsg)
{
  double left_wheel_velocity = 0, right_wheel_velocity = 0;
  double dtheta = 0, dx = 0, dy = 0, w = 0, dt;

  static double x = 0, y = 0, theta = 0;

  tf::Quaternion qt;
  tf::Vector3 vt;

  for(size_t k = 0; k <  2; k++)
  {
    if (recievedMsg -> name[k] == "left_motor")
    {
      left_wheel_velocity  = recievedMsg -> velocity[k];
    }
    else if (recievedMsg -> name[k] == "right_motor")
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
  odomMsg.twist.twist.angular.z   = dtheta/0.01;

  odomMsg.twist.twist.linear.x    = dx/0.01;
  odomMsg.twist.twist.linear.y    = dy/0.01;
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
