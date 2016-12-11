#include <kalman-filter/kalman-filter.h>
#include <catbot_localization/getIMU.h>
#include <tf/tf.h>

double wheel_radius = 0.0956;
double b  = 0.3195;
double bi = 1/0.3195;

using std::cout;
using std::endl;

void KalmanFilter::recursiveUpdate(const sensor_msgs::JointState::ConstPtr &odometryMsg)
{
	static double left_wheel_position = 0, right_wheel_position = 0, prev_right_wheel_position, prev_left_wheel_position ;
	double dtheta = 0, dx = 0, dy = 0;
	double ds, ds_l, ds_r;

	tf::Vector3 vt;
	tf::Quaternion qt;

	double dt     = (ros::Time::now() - lastTimeStamp).toSec();
	lastTimeStamp = ros::Time::now();

	for(size_t k = 0; k <  odometryMsg->name.size(); k++)
	{
	  if (odometryMsg -> name[k] == "left_motor")
	  {
		left_wheel_position  = odometryMsg -> position[k];
	  }
	  else if (odometryMsg -> name[k] == "right_motor")
	  {
		right_wheel_position = odometryMsg -> position[k];
	  }
	}

	ds_r = (right_wheel_position - prev_right_wheel_position) * wheel_radius;
	ds_l = (left_wheel_position  - prev_left_wheel_position) * wheel_radius;

	prev_left_wheel_position  = left_wheel_position;
	prev_right_wheel_position = right_wheel_position;

	ds		= (ds_r + ds_l)* 0.5;
	dtheta  = (ds_r - ds_l)* bi;

	dx      = ds * cos(updatedStates[2] + dtheta/2);
	dy      = ds * sin(updatedStates[2] + dtheta/2);

	predictedStates   = updatedStates  + Eigen::Vector3d(dx,dy,dtheta);
	getPoseIMU();

	Eigen::Matrix3d f_p = getJacobian_X(updatedStates[2], ds, dtheta);
	Eigen::MatrixXd f_d(3,2);
	f_d = getJacobian_U(updatedStates[2], ds, dtheta, bi);
	inputCovariance		= getCovarianceInput(fabs(ds_r),fabs(ds_l));

	predictedCovariance = f_p * positionCovariance * f_p.transpose() + f_d * inputCovariance * f_d.transpose();
	KalmanGain			= predictedCovariance * Eigen::Inverse<Eigen::Matrix3d>(predictedCovariance + measurementCovariance);

	updatedStates =  predictedStates + KalmanGain * (observation - predictedStates);
	positionCovariance	= predictedCovariance  - KalmanGain * predictedCovariance;


	cout << "dS_l: " << ds_l << " , " << " dS_r: " << ds_r << " And dt is: " << dt << endl;

	geometry_msgs::PoseStamped refinedPose;

	refinedPose.header = odometryMsg->header;
	refinedPose.header.stamp = ros::Time::now();
	refinedPose.header.frame_id  = "world";
	refinedPose.pose.position.x  = updatedStates[0];
	refinedPose.pose.position.y  = updatedStates[1];
	refinedPose.pose.position.z	 = 0;
	refinedPose.pose.orientation = tf::createQuaternionMsgFromYaw(updatedStates[2]);

	qt.setW(refinedPose.pose.orientation.w);
	qt.setX(refinedPose.pose.orientation.x);
	qt.setY(refinedPose.pose.orientation.y);
	qt.setZ(refinedPose.pose.orientation.z);

	vt.setX(refinedPose.pose.position.x);
	vt.setY(refinedPose.pose.position.y);
	vt.setZ(refinedPose.pose.position.z);

	const ros::Time current_time = ros::Time::now();
	const tf::Transform base_footprint_to_world( qt, vt );

	tf_br.sendTransform(tf::StampedTransform(base_footprint_to_world.inverse(),current_time,"base_footprint","kalman"));

	outputPublisher.publish(refinedPose);
}

void KalmanFilter::getPoseIMU()
{
	catbot_localization::getIMU imuPoseMsg;

	if (imuClient.call(imuPoseMsg))
	{
		observation(0) = imuPoseMsg.response.pose.position.x;
		observation(1) = imuPoseMsg.response.pose.position.y;

		tf::Quaternion qt;
		tf::quaternionMsgToTF(imuPoseMsg.response.pose.orientation,qt);

		observation(2) = tf::getYaw(qt);
	}
}

Eigen::Matrix3d KalmanFilter::getJacobian_X(double theta, double ds, double dtheta)
{
	Eigen::Matrix3d f_p(3,3);
	f_p(0,0) = 1;	f_p(0,1) = 0;	f_p(0,2) = -ds*sin(theta+0.5*dtheta);
	f_p(1,0) = 0;	f_p(1,1) = 1;	f_p(1,2) =  ds*cos(theta+0.5*dtheta);
	f_p(2,0) = 0;	f_p(2,1) = 0;	f_p(2,2) =  1;

	return f_p;
}

Eigen::MatrixXd KalmanFilter::getJacobian_U(double theta, double ds, double dtheta, double bi)
{
	Eigen::MatrixXd f(3,2);
	double c = cos(theta+0.5*dtheta), s = sin(theta+0.5*dtheta);

	f(0,0)	=	0.5*(c-ds*s*bi);	f(0,1)	=	0.5*(c+ds*s*bi);
	f(1,0)	=	0.5*(s+ds*c*bi);	f(1,1)	=	0.5*(s-ds*c*bi);
	f(2,0)	=	bi;					f(2,1)	=	-bi;

	return f;
}

Eigen::Matrix2d KalmanFilter::getCovarianceInput(double ds_r, double ds_l)
{
	Eigen::Matrix2d s_u(2,2);

	s_u(0,0) = 0.01*ds_r;	s_u(0,1) = 0		 ;
	s_u(1,0) = 0		 ;  s_u(1,1) = 0.01*ds_l;

	return s_u;
}

void KalmanFilter::initializeFilter(std::string publishTopicName, std::string subscribeTopicName, std::string imuServiceName)
{
	outputPublisher    = nodeHandle.advertise<geometry_msgs::PoseStamped>(publishTopicName,1);
	odometrySubscriber = nodeHandle.subscribe<sensor_msgs::JointState>(subscribeTopicName,1,&KalmanFilter::recursiveUpdate,this);
	imuClient		   = nodeHandle.serviceClient<catbot_localization::getIMU>(imuServiceName);
	updatedStates	   = Eigen::Vector3d(0,0,0);

	positionCovariance(0,0) = 0.0001; positionCovariance(0,1) = 0; positionCovariance(0,2) = 0;
	positionCovariance(1,0) = 0; positionCovariance(1,1) = 0.0001; positionCovariance(1,2) = 0;
	positionCovariance(2,0) = 0; positionCovariance(2,1) = 0; positionCovariance(2,2) = 0.0001;

	measurementCovariance(0,0) = 0.0625; measurementCovariance(0,1) = 0; measurementCovariance(0,2) = 0;
	measurementCovariance(1,0) = 0; measurementCovariance(1,1) = 0.0625; measurementCovariance(1,2) = 0;
	measurementCovariance(2,0) = 0; measurementCovariance(2,1) = 0; measurementCovariance(2,2) = 0.0001;
}
