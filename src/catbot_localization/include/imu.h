#ifndef IMU_H
#define IMU_H
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <catbot_localization/getIMU.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

std::default_random_engine generator;
std::normal_distribution<double> distribution(0,0.00625);

class IMU
{
public:
  IMU(std::string imuTopicName, std::string posInfoTopicName, int queueSize)
  {
	pubObject		 = nH.advertise<geometry_msgs::PoseStamped>(posInfoTopicName,queueSize);
	subObject		 = nH.subscribe<sensor_msgs::Imu>(imuTopicName,queueSize, &IMU::subCallback,this);
	lastTimeStamp	 = ros::Time::now();
	geometry_msgs::Pose newPose;
	newPose.position.x = 0;
	newPose.position.y = 0;
	newPose.orientation.x = 0;
	newPose.orientation.y = 0;
	newPose.orientation.z = 0;
	newPose.orientation.w = 1;
	poses.push_back(newPose);
  }
  void subCallback(const sensor_msgs::Imu::ConstPtr& recievedMsg)
  {

	tf::Vector3 vt;
	tf::Quaternion qt;

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

	geometry_msgs::PoseStamped newPose;
	newPose.header = recievedMsg->header;
	newPose.header.frame_id = "world";
	newPose.pose.position.x = x + distribution(generator);
	newPose.pose.position.y = y + distribution(generator);
	newPose.pose.orientation.x = orientation.x();
	newPose.pose.orientation.y = orientation.y();
	newPose.pose.orientation.z = orientation.z();
	newPose.pose.orientation.w = orientation.w();

	qt.setW(newPose.pose.orientation.w);
	qt.setX(newPose.pose.orientation.x);
	qt.setY(newPose.pose.orientation.y);
	qt.setZ(newPose.pose.orientation.z);

	vt.setX(newPose.pose.position.x);
	vt.setY(newPose.pose.position.y);
	vt.setZ(newPose.pose.position.z);

	pubObject.publish(newPose);
	poses.push_back(newPose.pose);
	if (poses.size() > 50)
	{
		poses.erase(poses.begin());
	}
	const ros::Time current_time = ros::Time::now();
	const tf::Transform base_footprint_to_world( qt, vt );

	tf_br.sendTransform(tf::StampedTransform(base_footprint_to_world.inverse(),current_time,"base_footprint","imu"));

  }

  bool srvCallback(catbot_localization::getIMU::Request& request, catbot_localization::getIMU::Response& response)
  {
	  response.pose = poses[poses.size()-1];
  }

  ros::NodeHandle nH;

private:
  ros::Subscriber subObject;
  ros::Publisher  pubObject;
  ros::Time lastTimeStamp, currTimeStamp;
  tf2::Quaternion orientation;
  tf2::Vector3 accBody, angVelBody;
  tf2::Vector3 accInertial, angVelInertial;
  std::vector<geometry_msgs::Pose> poses;
  tf::TransformBroadcaster tf_br;
  double  x=0,  y=0, theta=0,
         vx=0, vy=0,     w=0;
};

#endif // IMU_H
