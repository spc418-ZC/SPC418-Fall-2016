#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <unistd.h>
#include <iomanip>
#include <iostream>
#include <termios.h>
#include <sys/select.h>
#include <sys/types.h>
#include <geometry_msgs/Twist.h>

#define NOMINAL_LINEAR_VEL 0.25
#define NOMINAL_ANGULAR_VEL	M_PI/4
using std::cout;
using std::endl;

char getKey(fd_set rfds);
void help();

int main(int argc, char **argv)
{
  double nominal_linear_vel  = 0;
  double nominal_angular_vel = 0;

  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(STDIN_FILENO,&rfds);

  char inputKey;

  termios term_settings;
  tcgetattr(STDIN_FILENO,&term_settings);
  term_settings.c_lflag = ISIG |TOSTOP|ECHOK|IEXTEN|~ICANON;
  tcsetattr(STDIN_FILENO,TCSANOW,&term_settings);


  // Set up ROS.
  ros::init(argc, argv, "kinematic_model");
  ros::NodeHandle DiffDriveNode;
  ros::Publisher cmd_VelTopic  = DiffDriveNode.advertise<geometry_msgs::Twist>("velocity_command",10);
  ros::Rate rate_controller(30);
  ros::Duration waitPeriod(0);

  while(ros::ok())
  {
    inputKey = getKey(rfds);

	switch (inputKey)
    {
      case '/':
      {
        term_settings.c_lflag = ECHO|ISIG|TOSTOP|ECHOK|IEXTEN|ICANON;
        tcsetattr(STDIN_FILENO,TCSANOW,&term_settings);
        cout << "Quit command has been issued ... Quitting" << endl;
        return 0;
	  }
      case '2':
      {
		nominal_linear_vel  = -NOMINAL_LINEAR_VEL;
		nominal_angular_vel = 0;
		waitPeriod = ros::Duration(1/NOMINAL_LINEAR_VEL);
        break;
      }
      case '4':
      {
		nominal_linear_vel  = 0;
		nominal_angular_vel = NOMINAL_ANGULAR_VEL;
		waitPeriod = ros::Duration(1/NOMINAL_ANGULAR_VEL);
        break;
	  }
	  case '6':
	  {
		nominal_linear_vel  = 0;
		nominal_angular_vel = -NOMINAL_ANGULAR_VEL;
		waitPeriod = ros::Duration(1/NOMINAL_ANGULAR_VEL);
		break;
      }
      case '8':
	  {
		nominal_linear_vel  = NOMINAL_LINEAR_VEL;
		nominal_angular_vel = 0;
		waitPeriod = ros::Duration(1/NOMINAL_LINEAR_VEL);
		break;
	  }
	}

	geometry_msgs::Twist vel_command;

	vel_command.linear.x = nominal_linear_vel;
	vel_command.angular.z = nominal_angular_vel;
	cmd_VelTopic.publish(vel_command);
	waitPeriod.sleep();

	nominal_linear_vel  = 0;
	nominal_angular_vel = 0;
	vel_command.linear.x = nominal_linear_vel;
	vel_command.angular.z = nominal_angular_vel;
	cmd_VelTopic.publish(vel_command);

	rate_controller.sleep();
  }
  return 0;
}


char getKey(fd_set rfds)
{
    char key_buff[1];
    struct timespec tv;
    tv.tv_sec  = 0.001;
    tv.tv_nsec = 0;
    pselect(STDIN_FILENO+1,&rfds,NULL,NULL,&tv,NULL);

    if (FD_ISSET(STDIN_FILENO, &rfds))
    {
        if (fread(key_buff,sizeof(key_buff),1,stdin))
        {
            return key_buff[0];
        }
        else
        {
            return '\0';
        }
    }
}
