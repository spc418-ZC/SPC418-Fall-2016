#ifndef IMU_H
#define IMU_H
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <assignment_2/Pose2D.h>
#include <tf2/LinearMath/Quaternion.h>

class IMU
{
public:
  IMU(std::string imuTopicName, std::string posInfoTopicName, int queueSize)
  {
    subObject     = nH.subscribe<sensor_msgs::Imu>(imuTopicName,queueSize, &IMU::subCallback,this);
    pubObject     = nH.advertise<assignment_2::Pose2D>(posInfoTopicName, queueSize);
    lastTimeStamp = ros::Time::now();
  }
  void subCallback(const sensor_msgs::Imu::ConstPtr& recievedMsg)
  {
    currTimeStamp = recievedMsg->header.stamp;
    double dt     = (currTimeStamp-lastTimeStamp).toSec();
    lastTimeStamp = currTimeStamp;

    orientation   = tf2::Quaternion(recievedMsg->orientation.x,
                                    recievedMsg->orientation.y,
                                    recievedMsg->orientation.z,
                                    recievedMsg->orientation.w);

    accBody       = tf2::Vector3(recievedMsg->linear_acceleration.x,
                                 recievedMsg->linear_acceleration.y,
                                 recievedMsg->linear_acceleration.z);

    angVelBody    = tf2::Vector3(recievedMsg->angular_velocity.x,
                                 recievedMsg->angular_velocity.y,
                                 recievedMsg->angular_velocity.z);

    accInertial    = tf2::quatRotate(orientation,accBody);
    angVelInertial = tf2::quatRotate(orientation,angVelBody);

    vx  += ((fabs(accInertial.x()) > 1e-3)? accInertial.x() : 0) * dt;
    vy  += ((fabs(accInertial.y()) > 1e-3)? accInertial.y() : 0) * dt;

    x   += ((fabs(vx) > 1e-4)? vx : 0) * dt;
    y   += ((fabs(vy) > 1e-4)? vy : 0) * dt;

    theta = orientation.getAngle();
    w     = angVelInertial.getZ();

    assignment_2::Pose2D pose2D;
    pose2D.x = x;
    pose2D.y = y;
    pose2D.theta = theta;

    pubObject.publish(pose2D);
  }

private:
  ros::NodeHandle nH;
  ros::Publisher pubObject;
  ros::Subscriber subObject;
  ros::Time lastTimeStamp, currTimeStamp;
  tf2::Quaternion orientation;
  tf2::Vector3 accBody, angVelBody;
  tf2::Vector3 accInertial, angVelInertial;
  double  x=0,  y=0, theta=0,
         vx=0, vy=0,     w=0;
};

#endif // IMU_H
