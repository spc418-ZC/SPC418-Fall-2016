#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <iomanip>
#include <termios.h>
#include <sys/select.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>

using std::cout;
using std::endl;

char getKey(fd_set);
void help();

int main(int argc, char **argv)
{
    double linear_vel=0,angular_vel=0;
    help();
    // Opening the terminal stdin and attaching it to a file descriptor
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO,&rfds);
    char key;
    // Getting current terminal settings for resetting
    termios term_settings;
    tcgetattr(STDIN_FILENO,&term_settings);
    term_settings.c_lflag = ISIG |TOSTOP|ECHOK|IEXTEN|~ICANON;
    tcsetattr(STDIN_FILENO,TCSANOW,&term_settings);

    //Initializing ROS
    ros::init(argc, argv, "catbot_teleop");
    ros::NodeHandle node;
    ros::Publisher teleop_pub = node.advertise<geometry_msgs::Twist>("/catbot/cmd_vel",5);
    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        key = getKey(rfds);
        switch (key)
        {
            case 'r':
            {
                term_settings.c_lflag = ECHO|ISIG|TOSTOP|ECHOK|IEXTEN|ICANON;
                tcsetattr(STDIN_FILENO,TCSANOW,&term_settings);
                cout << "Quit command has been issued ... Quitting" << endl;
                return 0;
            }
            case 'w':
            {
                linear_vel   =  0.2;
                break;
            }
            case 'a':
            {
                angular_vel  =  0.1;
                break;
            }
            case 's':
            {
                linear_vel   = -0.2;
                break;
            }
            case 'd':
            {
                angular_vel  = -0.1;
                break;
            }
            case '+':
            {
                linear_vel  *= 1.1;
                break;
            }
            case '-':
            {
                linear_vel  *= 0.9;
                break;
            }
            case ',':
            {
                angular_vel *= 1.1;
                break;
            }
            case '.':
            {
                angular_vel *= 0.9;
                break;
            }
            case ' ':
            {
                linear_vel   = 0;
                angular_vel  = 0;
                break;
            }
            case '/':
            {
                angular_vel  = 0;
            }
            case '_':
            {
                linear_vel   = 0;
            }
//            case '\0':
//            {
//              linear_vel = 0;
//              angular_vel = 0;
//            }
        }

        geometry_msgs::Twist      vel_command;

        vel_command.linear.x  = linear_vel;
        vel_command.linear.y  = 0;
        vel_command.linear.z  = 0;
        vel_command.angular.x = 0;
        vel_command.angular.y = 0;
        vel_command.angular.z = angular_vel;

        teleop_pub.publish(vel_command);

        loop_rate.sleep();
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
    cout << "Hello, This node is responsible for Keyboard teleoperation for CATBot" << endl;
    cout << "---------------------------------------------------------------------" << endl;
    cout << "Key bindings are described below:" << endl;
    cout << "-----------------" << endl;
    cout << " w : moves forward" << endl;
    cout << " s : moves backward" << endl;
    cout << " d : moves right" << endl;
    cout << " a : moves left" << endl;

    cout << " + : increase linear speed by 10%" << endl;
    cout << " - : decrease linear speed by 10%" << endl;
    cout << " , : increase angular speed by 10%" << endl;
    cout << " . : decrease angular speed by 10%" << endl;
    cout << " r : reset terminal settings and quit node" << endl;
}
