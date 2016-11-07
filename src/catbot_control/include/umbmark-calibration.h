#ifndef UMBMARKCALIBRATION_H
#define UMBMARKCALIBRATION_H

#include <PublisherSubscriber.h>
#include <tf2/LinearMath/Quaternion.h>

PublisherSubscriber<std_msgs::Float64, geometry_msgs::PoseStamped>::PublisherSubscriber(){ }

class UMBmarkMethod : protected PublisherSubscriber<std_msgs::Float64, geometry_msgs::PoseStamped>
{
public:
  UMBmarkMethod::subscriberCallback(const geometry_msgs::PoseStamped_::ConstPtr &recievedMsg)
  {
    err_x = goal_x - recievedMsg->pose.position.x;
    err_y = goal_y - recievedMsg->pose.position.y;
    tf::Quaternion qt;
    tf::quaternionMsgToTF(recievedMsg->pose.orientation,qt);

  }
protected:
  double goal_x;
  double goal_y;
  double goal_theta;
  double err_x;
  double err_y;
  double err_theta;
};

#endif // UMBMARKCALIBRATION_H
