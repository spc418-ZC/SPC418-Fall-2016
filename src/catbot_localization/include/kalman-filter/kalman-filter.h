#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>

class KalmanFilter
{
	protected:
		ros::NodeHandle		nodeHandle;
		ros::Publisher		outputPublisher;
		ros::Subscriber		odometrySubscriber;
		ros::ServiceClient	imuClient;
		Eigen::Matrix3d		positionCovariance;
		Eigen::Matrix3d		predictedCovariance;
		Eigen::Matrix2d		inputCovariance;
		Eigen::Matrix3d		measurementCovariance;
		Eigen::Matrix3d		motionJacobian_X;
		Eigen::MatrixXd		motionJacobian_U;
		Eigen::Matrix3d		measurmentJacobian;
		Eigen::Vector3d		predictedStates;
		Eigen::Vector3d		observation;
		Eigen::Vector3d		updatedStates;
		Eigen::Matrix3d		KalmanGain;
		ros::Time			lastTimeStamp;
		tf::TransformBroadcaster	tf_br;
	public:
		void recursiveUpdate(const sensor_msgs::JointState::ConstPtr& odometryMsg);
		void initializeFilter(std::__cxx11::string publishTopicName, std::__cxx11::string subscribeTopicName, std::__cxx11::string imuServiceName);
		void getPoseIMU();
		void updateStates();
		void publishStates();
		void calculateKalmanGain();
		void propagateCovariance();

		Eigen::Matrix3d getJacobian_X(double theta, double ds, double dtheta);
		Eigen::MatrixXd getJacobian_U(double theta, double ds, double dtheta, double bi);
		Eigen::Matrix2d getCovarianceInput(double ds_r, double ds_l);
};

#endif // KALMANFILTER_H
