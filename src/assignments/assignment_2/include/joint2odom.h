#ifndef JOINT2ODOM_H
#define JOINT2ODOM_H

#include <publisher_subscriber.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <math.h>

double wheel_separation = 0.16 * 2; //so we can use it in all functions
double wheel_diameter = 0.98;

template<>
void Publisher_Subscriber<nav_msgs::Odometry , sensor_msgs::JointState>::subscriberCallback(const sensor_msgs::JointState::ConstPtr& receivedMsg)
{
 // INITIALIZATION
  double right_wheel_position= 0, right_wheel_velocity = 0, left_wheel_position = 0, left_wheel_velocity = 0, velocity_cg = 0;
  static double w = 0, dx = 0, dy = 0, dtheta=0, dt=0, x=0, y=0, theta=0;
  ros::Time prev_time = ros::Time::now();

  //initialize static variables of seconds and nano seconds

    dt = 0.01; //(prev_time - receivedMsg->header.stamp).toSec();
    
    for(size_t k = 0; k <  2 ; k++) //or from k = 0 to k = 1
  {
      //check if name field in receivedMsg is left_motor
      //take the position and velocity fields and assign them to corresponding wheel
    if (receivedMsg -> name[k] == "left_motor")
    {
      left_wheel_position  = receivedMsg -> position[k];
      left_wheel_velocity  = receivedMsg -> velocity[k];
    }
    else if (receivedMsg -> name[k] == "right_motor")
    {
      right_wheel_position = receivedMsg -> position[k];
      right_wheel_velocity = receivedMsg -> velocity[k];
    }
  }

    //Differential Drive Kinematics
    w = (right_wheel_velocity - left_wheel_velocity) * wheel_diameter / wheel_separation; //angular velocity
    
    velocity_cg = (right_wheel_velocity + left_wheel_velocity) / 2; //velocity of centre of mass (assumed to be symmetric

    dtheta = w * dt;
    theta += (fabs(dtheta) > 0.0001)? dtheta : 0;

    dx     = velocity_cg * dt * cos(theta);
    dy     = velocity_cg * dt * sin(theta);

    x     += (fabs(dx) > 0.00001)? dx : 0;
    y     += (fabs(dy) > 0.00001)? dy : 0;

    tf::Quaternion qt;
    tf::Vector3 vt;

    qt.setRPY(0,0,theta);
    vt.setX(x);
    vt.setY(y);
    vt.setZ(0);

    nav_msgs::Odometry odomMsg;
    geometry_msgs::Quaternion quatMsg = tf::createQuaternionMsgFromYaw(theta);

    std_msgs::Header header;

    header.stamp    = ros::Time::now();
    header.frame_id = "odom";

    odomMsg.header = header;
    odomMsg.child_frame_id = "base_footprint";

    odomMsg.pose.pose.position.x  = x;
    odomMsg.pose.pose.position.y  = y;
    odomMsg.pose.pose.position.z  = 0;

    odomMsg.pose.pose.orientation = quatMsg;

    odomMsg.twist.twist.linear.x  = dx/dt;
    odomMsg.twist.twist.linear.y  = dy/dt;
    odomMsg.twist.twist.angular.z = w;

    tf::Transform transformation(qt,vt);
    tfBroad.sendTransform(tf::StampedTransform(transformation,header.stamp,"odom","base_footprint"));

    publisherObject.publish(odomMsg);

}


#endif // JOINT2ODOM_H
