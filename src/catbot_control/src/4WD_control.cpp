#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

class _4WD_controller
{
  public:
    _4WD_controller()
    {
      // Initializting the publishers for each of the controllers
      br_controller = WD.advertise<std_msgs::Float64>("/catbot/back_right_wheel_effort_controller/command",5);
      bl_controller = WD.advertise<std_msgs::Float64>("/catbot/back_left_wheel_effort_controller/command",5);
      fr_controller = WD.advertise<std_msgs::Float64>("/catbot/front_right_wheel_effort_controller/command",5);
      fl_controller = WD.advertise<std_msgs::Float64>("/catbot/front_left_wheel_effort_controller/command",5);

      // Initializing the command topic subscriber
      // The callback function attached to the subscriber topic is a method of this class and the syntax below is different from a normal subscriber object
      // Check the documentation
      cmd_subscriber= WD.subscribe("/catbot/cmd_vel", 5, &_4WD_controller::cmd_callback, this);
      ros::spin();
    }

    // This method creates and publishes the control signals to their respective topics
    void publish_controls()
    {
      std_msgs::Float64 br_control_signal;  br_control_signal.data = br_motor;
      std_msgs::Float64 bl_control_signal;  bl_control_signal.data = bl_motor;
      std_msgs::Float64 fr_control_signal;  fr_control_signal.data = fr_motor;
      std_msgs::Float64 fl_control_signal;  fl_control_signal.data = fl_motor;

      br_controller.publish(br_control_signal);
      bl_controller.publish(bl_control_signal);
      fr_controller.publish(fr_control_signal);
      fl_controller.publish(fl_control_signal);
    }

    // This method calculates the control signals and stores them in the class's private attributes
    void calculate_controls()
    {
      br_motor = 5*v;      br_motor -= 20*w;
      fr_motor = 5*v;      fr_motor -= 20*w;
      bl_motor = 5*v;      bl_motor += 20*w;
      fl_motor = 5*v;      fl_motor += 20*w;
    }

    //This is used as the callback function attached to the subscriber object inside this class
    void cmd_callback(const geometry_msgs::Twist::ConstPtr& msg)
    {
      v = msg->linear.x;
      w = msg->angular.z;
      calculate_controls();
      publish_controls();
    }

  private:
    double v=0,w=0; // Linear and Angular Velocity
    double br_motor,bl_motor,fr_motor,fl_motor; // Motor Signals
    ros::Publisher br_controller, bl_controller, fr_controller, fl_controller; // Control Signal Publishesrs
    ros::Subscriber cmd_subscriber; // Command Subscriber
    ros::NodeHandle WD;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "four_WD_control");
  _4WD_controller controller;
  return 0;
}
