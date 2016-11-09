#ifndef UMBMARKCALIBRATION_H
#define UMBMARKCALIBRATION_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>

class UMBmarkMethod
{
public:
  UMBmarkMethod(std::string leftMotorTopicName, std::string rightMotorTopicName, std::string subscribeTopicName, int queueSize)
  {
    left_motor_publisher  = nH.advertise<std_msgs::Float64>(leftMotorTopicName,queueSize);
    right_motor_publisher = nH.advertise<std_msgs::Float64>(rightMotorTopicName,queueSize);
    subscriberObject      = nH.subscribe<geometry_msgs::PoseStamped>(subscribeTopicName,queueSize,&UMBmarkMethod::subscriberCallback,this);
  }

  void subscriberCallback(const geometry_msgs::PoseStamped::ConstPtr &recievedMsg)
  {
    theta = 2 * acos(recievedMsg->pose.orientation.w);

//    if (recievedMsg->pose.orientation.z < 0)
//    {
//      theta = 2 * M_PI - theta;
//    }
    err_x = goal_x - recievedMsg->pose.position.x;
    err_y = goal_y - recievedMsg->pose.position.y;

    computeControlSignal();

    std_msgs::Float64 rmCS, lmCS;

    rmCS.data = rmControlSignal;
    lmCS.data = lmControlSignal;

    left_motor_publisher.publish(lmCS);
    right_motor_publisher.publish(rmCS);

    if((fabs(psi) < tolerance) && (fabs(r) < tolerance))
    {
      goal_x = 5;
      goal_y = -5;
    }
  }

  void computeControlSignal()
  {
    r     = sqrt(err_x*err_x   + err_y*err_y);
    psi   = atan2(err_y,err_x) - theta;

    ROS_INFO("R is: %f, Psi is: %f, Theta is : %f", r, psi, theta);
    dr    = r   - prev_r;
    dpsi  = psi - prev_psi;

    ir   += r   * dt;
    ipsi += psi * dt;

    rmControlSignal = ((fabs(psi) < (tolerance*2))? 1 : 0) * (kp * r + kd * dr + ki * ir * 0) + ((fabs(psi) > (tolerance))? 1 : 0) * (kp * psi + kd * dpsi + ki * ipsi);
    lmControlSignal = ((fabs(psi) < (tolerance*2))? 1 : 0) * (kp * r + kd * dr + ki * ir * 0) - ((fabs(psi) > (tolerance))? 1 : 0) * (kp * psi + kd * dpsi + ki * ipsi);

    prev_r =r;
    prev_psi = psi;
  }

protected:

  ros::NodeHandle nH;
  ros::Publisher  right_motor_publisher;
  ros::Publisher   left_motor_publisher;
  ros::Subscriber subscriberObject;

  double goal_x = 0    , goal_y = 7.5    , goal_z = 0    ;
  double prev_err_z = 0;

  double err_x     , err_y     , err_z     ;
  double kp = 1   , kd = 0.1   , ki= 0.00 ;

  double derr_z = 0, ierr_z = 0;
  double r, prev_r = 0, psi, prev_psi = 0;
  double dr = 0, dpsi = 0, ir, ipsi = 0;
  double rmControlSignal, lmControlSignal;

  double tolerance = 0.02, dt = 0.025, theta = 0;
};

#endif // UMBMARKCALIBRATION_H
