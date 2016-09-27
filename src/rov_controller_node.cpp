#include "rov_controller_node.h"
#include "ros/ros.h"
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <serial/serial.h>
#include "std_msgs/String.h"
#include <string>
#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

using namespace std;



double mapValues(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



int analogWrite(int fd, unsigned char channel, unsigned short target)
{	
  const char * device = "/dev/ttyACM0";
  fd = open(device, O_RDWR | O_NOCTTY);

  unsigned char command[] = {0x84, channel,static_cast<unsigned char>(target & 0x7F),static_cast<unsigned char>((target >> 7) & 0x7F)};
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  close(fd);
  return 0;
}

//enum trusters {top_left, top_right, middle_left, middle_middle, middle_right, bottom};

void set_speed(unsigned int truster, float value){ // trusters from the enum above, value -100 to 100
	value = mapValues(value, -2, 2 , 4000, 8000);
	analogWrite(fd,truster, value);
}



void integrate_cmd_vel(const ros::TimerEvent& integrate_timer){
	if(close_loop){
		float speed_sensitivity = 0.1;
		x_ref += speed_sensitivity*x_ref_dot;
		y_ref += speed_sensitivity*y_ref_dot;
		z_ref += speed_sensitivity*z_ref_dot;

		x_angle_ref += speed_sensitivity*x_angle_ref_dot;
		y_angle_ref += speed_sensitivity*y_angle_ref_dot;
		z_angle_ref += speed_sensitivity*z_angle_ref_dot;


	/*
		cout << "CLOSED LOOP ENABLED" << endl;
		cout <<"x: \t" << x_ref;
		cout << endl << "y: \t" << y_ref;
		cout << endl << "z: \t" << z_ref << endl << endl;
	*/
	}

}

void set_truster_configuration_and_speed(double x_speed,double y_speed, double z_speed, double x_angular_speed, double y_angular_speed, double z_angular_speed){
	//Linear.x	
	float top_left_speed = 0;
	float top_right_speed = 0;
	float middle_left_speed = 0;
	float middle_middle_speed = -1*x_speed*2;
	float middle_right_speed = 0;
	float bottom_speed = 0;
	

	//Linear.y
	top_left_speed += 0;
	top_right_speed += 0;
	middle_left_speed +=y_speed;
	middle_middle_speed += 0;
	middle_right_speed += -1*y_speed;
	bottom_speed += 0;

	//Linear.z
	top_left_speed += z_speed;
	top_right_speed += -1*z_speed;
	middle_left_speed += 0;
	middle_middle_speed += 0;
	middle_right_speed += 0;
	bottom_speed += z_speed;

	//Angular.x	
	top_left_speed += -1*x_angular_speed;
	top_right_speed += -1*x_angular_speed;
	middle_left_speed += 0;
	middle_middle_speed += 0;
	middle_right_speed += 0;
	bottom_speed += x_angular_speed;
	
	//Angular.y
	top_left_speed += -1*y_angular_speed;
	top_right_speed += y_angular_speed;
	middle_left_speed +=0;
	middle_middle_speed += 0;
	middle_right_speed += 0;
	bottom_speed += 0;

	//Angular.z
	top_left_speed += 0;
	top_right_speed += 0;
	middle_left_speed += z_angular_speed;
	middle_middle_speed += 0;
	middle_right_speed += z_angular_speed;
	bottom_speed += 0;

	set_speed(top_left, top_left_speed);
	set_speed(top_right, top_right_speed);
	set_speed(middle_left, middle_left_speed);
	set_speed(middle_middle, middle_middle_speed);
	set_speed(middle_right, middle_right_speed);
	set_speed(bottom, bottom_speed);


}



void joy_callback(const geometry_msgs::Twist::ConstPtr& msg){
		
	if(!close_loop){
		set_truster_configuration_and_speed(msg->linear.x ,msg->linear.z, msg->linear.y, msg->angular.x, msg->angular.y, msg->angular.z);
	}
	else{
		x_ref_dot = msg->linear.x;
		y_ref_dot = msg->linear.y;
		z_ref_dot = msg->linear.z;

		x_angle_ref_dot = msg->angular.x;
		y_angle_ref_dot = msg->angular.y;
		z_angle_ref_dot = msg->angular.z;
	}
}



void PIDCallback(const ros::TimerEvent& event){

	

}


void camera_odom_callback(const nav_msgs::Odometry::ConstPtr& odom){
	tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double x_angle, y_angle, z_angle;
	m.getRPY(y_angle, x_angle, z_angle);
	if(reset_odometry){
		x_ref = odom->pose.pose.position.x;
		y_ref = odom->pose.pose.position.y;
		z_ref = odom->pose.pose.position.z;
		cout << reset_odometry << endl;
		reset_odometry = false;
	}
	
	double x_force = kp_linear_x*(x_ref-odom->pose.pose.position.x);	
	double y_force = kp_linear_y*(y_ref-odom->pose.pose.position.y);	
	double z_force = kp_linear_z*(z_ref-odom->pose.pose.position.z);	
	cout << "x_force: \t " << x_force << endl;
	cout << "y_force: \t " << y_force << endl;
	cout << "z_force: \t " << z_force << endl << endl << endl;
	if(close_loop)	
		set_truster_configuration_and_speed(x_force ,y_force, z_force, 0, 0, 0);
}


void switch_loop_state(const std_msgs::Bool::ConstPtr& state){
	close_loop = state->data;
	cout << "Setting close_loop to: \t" << close_loop << endl;
	if(close_loop){
		//camera_odom_sub = n.subscribe("/odom", 1, camera_odom_callback);
		reset_odometry = true;
	}	
	else{
		// camera_odom_sub.shutdown();
	}
}


int main(int argc, char **argv)
{
  if(!ports_initialized){
    const char * device = "/dev/ttyACM0";
    fd = open(device, O_RDWR | O_NOCTTY);
    ports_initialized = 1;
  }

  ros::init(argc, argv, "rov_controller");
  ros::NodeHandle n;
  ros::Subscriber camera_odom_sub;
  ros::Subscriber nav_msg_sub = n.subscribe("/cmd_vel", 1, joy_callback);
  ros::Subscriber closed_loop_sub = n.subscribe("/close_loop", 1, switch_loop_state);
  ros::Timer timer = n.createTimer(ros::Duration(0.05), PIDCallback);
  camera_odom_sub = n.subscribe("/camera/odom", 1, camera_odom_callback);
  ros::Timer integrate_timer = n.createTimer(ros::Duration(0.05), integrate_cmd_vel);

  ros::spin();

  //last wishes, so that the ROV stops when killing the program
  set_truster_configuration_and_speed(0, 0, 0, 0, 0, 0);
  close(fd);	

  return 0;

}




