#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <unistd.h>
#include <iomanip>
#include <iostream>
#include <termios.h>
#include <sys/select.h>
#include <sys/types.h>

using std::cout;
using std::endl;

char getKey(fd_set rfds);
void help();

int main(int argc, char **argv)
{
  cout << "\n \n \n \n \n \n \n \n \n \n \n \n" << endl;
  help();

  double right_motor_command = 0;
  double left_motor_command  = 0;
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

  ros::Publisher RightMotorCMD = DiffDriveNode.advertise<std_msgs::Float64>("/catbot/right_motor_controller/command",10);
  ros::Publisher LeftMotorCMD  = DiffDriveNode.advertise<std_msgs::Float64>("/catbot/left_motor_controller/command",10);

  ros::Rate rate_controller(30);

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
      case '1':
      {
        if ((left_motor_command== 0) || (left_motor_command < 0))
        {
          left_motor_command = -0.2;
        }
        else
        {
          left_motor_command*= 1.2;
        }
        nominal_linear_vel  = (left_motor_command + right_motor_command)*0.5;
        nominal_angular_vel = (right_motor_command - left_motor_command)*0.314;
        break;
      }
      case '2':
      {
        if ((nominal_linear_vel == 0) || (nominal_linear_vel > 0))
        {
          nominal_linear_vel = -0.2;
        }
        else
        {
          nominal_linear_vel*= 1.2;
        }
        right_motor_command = nominal_linear_vel + nominal_angular_vel * 1.5923;
        left_motor_command  = nominal_linear_vel - nominal_angular_vel * 1.5923;
        break;
      }
      case '3':
      {
        if ((right_motor_command== 0) || (right_motor_command < 0))
        {
          right_motor_command = -0.2;
        }
        else
        {
          right_motor_command*= 1.2;
        }
        nominal_linear_vel   = (left_motor_command + right_motor_command)*0.5;
        nominal_angular_vel  = (right_motor_command - left_motor_command)*0.314;
        break;
      }
      case '4':
      {
        if ((nominal_angular_vel == 0) || (nominal_angular_vel < 0))
        {
          nominal_angular_vel = 0.20;
        }
        else
        {
          nominal_angular_vel*= 1.2;
        }
        right_motor_command = nominal_linear_vel + nominal_angular_vel * 1.5923;
        left_motor_command  = nominal_linear_vel - nominal_angular_vel * 1.5923;
        break;
      }
      case '5':
      {
        nominal_linear_vel  = 0;
        nominal_angular_vel = 0;
        left_motor_command  = 0;
        right_motor_command = 0;
        break;
      }
      case '6':
      {
        if ((nominal_angular_vel == 0) || (nominal_angular_vel > 0))
        {
          nominal_angular_vel = -0.2;
        }
        else
        {
          nominal_angular_vel*= 1.2;
        }
        right_motor_command = nominal_linear_vel + nominal_angular_vel * 1.5923;
        left_motor_command  = nominal_linear_vel - nominal_angular_vel * 1.5923;
        break;
      }
      case '7':
      {
        if ((left_motor_command== 0) || (left_motor_command < 0))
        {
          left_motor_command = 0.2;
        }
        else
        {
          left_motor_command*= 1.2;
        }
        nominal_linear_vel  = (left_motor_command + right_motor_command) * 0.5;
        nominal_angular_vel = (right_motor_command - left_motor_command) * 0.314;
        break;
      }
      case '8':
      {
        if ((nominal_linear_vel == 0) || (nominal_linear_vel < 0))
        {
          nominal_linear_vel = 0.2;
        }
        else
        {
          nominal_linear_vel*= 1.2;
        }
        right_motor_command = nominal_linear_vel + nominal_angular_vel * 1.5923;
        left_motor_command  = nominal_linear_vel - nominal_angular_vel * 1.5923;
        break;
      }
      case '9':
      {
        if ((right_motor_command== 0) || (right_motor_command < 0))
        {
          right_motor_command = 0.2;
        }
        else
        {
          right_motor_command*= 1.2;
        }
        nominal_linear_vel  = (left_motor_command + right_motor_command) * 0.5;
        nominal_angular_vel = (right_motor_command - left_motor_command) * 0.314;
        break;
      }
    }
    std_msgs::Float64 right_wheel_vel,left_wheel_vel;

    right_wheel_vel.data = right_motor_command * 5;
    left_wheel_vel.data  = left_motor_command * 5 ;

    RightMotorCMD.publish(right_wheel_vel);
    LeftMotorCMD.publish(left_wheel_vel);

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
void help()
{
    char controls[] = {'w','a','s','d','+','-',',','.'};
    cout << "Hello, This node is responsible for Keyboard teleoperation for CATBot for Kinematics Model demo" << endl;
    cout << "---------------------------------------------------------------------" << endl;
    cout << "Key bindings are described below:" << endl;
    cout << "-----------------" << endl;
    cout << " 8 : increase forward linear velocity" << endl;
    cout << " 2 : increase bacward linear velocity" << endl;
    cout << " 6 : increase clockwise angular velocity" << endl;
    cout << " 4 : increase counter-clockwise angular velocity" << endl;
    cout << " 5 : set all velocities to zero and stop wheels" << endl;
    cout << " 7 : increase forward left wheel velocity" << endl;
    cout << " 1 : decrease backward left wheel velocity" << endl;
    cout << " 9 : increase right forward wheel velocity" << endl;
    cout << " 3 : increase right backward wheel velocity" << endl;
    cout << " / : shutdown this node" << endl;
}
