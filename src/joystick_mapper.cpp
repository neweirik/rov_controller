#include "ros/ros.h"
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <time.h>
#include "std_msgs/String.h"
#include <string>
#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

using namespace std;
ros::Publisher velocity_pub; 
ros::Publisher close_loop_pub; 

std_msgs::Bool close_loop;



double mapValues(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void mapper_callback(const sensor_msgs::Joy::ConstPtr& msg){
	geometry_msgs::Twist vel_message;

	vel_message.angular.z = msg->axes[0]; //left joystick x-axes
 	vel_message.linear.z = msg->axes[1]; //left joystick y-axes
	vel_message.linear.x = -1*msg->axes[3]; //right joystick x-axes
	vel_message.linear.y = msg->axes[4]; //right joystick y-axes
	
	if(msg->buttons[8] == true){
		close_loop.data = !close_loop.data;	
		close_loop_pub.publish(close_loop);
		if(close_loop.data)
			cout << "Closed loop tracking ENABLED!" << endl;
		else
			cout << "Close loop tracking DISABLED!" << endl << endl;
		usleep(1000*200);
	}
	
	velocity_pub.publish(vel_message);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_remapper");
  ros::NodeHandle n;
  ros::Subscriber nav_msg_sub = n.subscribe("/joy", 1, mapper_callback);
  velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
  close_loop_pub = n.advertise<std_msgs::Bool>("/close_loop", 10);
  ros::spin();

  return 0;

}



