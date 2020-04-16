#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <iostream>
#include <cmath>

/*functions declarations*/
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
void clbk_odom(const nav_msgs::Odometry::ConstPtr& msg);
void fix_yaw(double x_d, double y_d, double z_d);
void change_state(int next_state);
void state_changer(geometry_msgs::Point des_pos);
double yaw_check(geometry_msgs::Point des_pos);


/*global variables*/
ros::Publisher motion_pub = {};
double yaw_ = 0;
double yaw_precision_ = cmath::pi;
double dist_precision = 0.3;
geometry_msgs::Point position_;
geometry_msgs::Point desired_position_;
desired_position_.x = -3; //
desired_position_.y =  7; //objective location (x,y,z)
desired_position_.z =  0; //
int state_ = 0;	//machine "decision" state
int obstacle_detected_ = 0;

/*main function*/
int main(int argc, char **argv)
{
	/*initializing the various topic subscriptions and publishes*/
	ros::init(argc, argv, "reading_laser");
	
	ros::NodeHandle n;
	
	::motion_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	ros::Subscriber motion_sub = n.subscribe("/m2wr/laser/scan", 100, clbk_laser);
	
	ros::Subsriber odom_sub = n.subscribe("/odom", 100, cblk_odom);
	
	/*finite state machine*/
	state_changer(desired_position_);
	
	if(state == 0)
	{
		go_straight_ahead(desired_position_);
	}
	else if(state == 1)
	{
		fix_yaw(desired_position_);
	}
	else if(state == 2)
	{
		get_over(desired_position_);
	}
	else if(state == 3)
	{
		done();
		return 0;
	}
	else
	{
		ROS_INFO("Unknown state!");
		return 0;
	}
	
	ros::spin();
	
	return 0;
	
}
/*functions definitions*/
/*yaw_check: determines the current yaw error*/
void state_changer(geometry_msgs::Point des_pos)
{
	double err_yaw = 0.0;
	err_yaw = yaw_check(des_pos);
	
	//if we arrived that the objective we finish the mission
	if(err_position <= dist_precision)
		change_state(3);
	
	//check if the yaw is correct, if not, we fix it
	if(cmath::fabs(err_yaw) <= yaw_precision_)
	{
		//case 'a': there is no obstacle -> go fwd
		if(obstacle_detected_ == 0)
			change_state(0);	//go straight fwd
		//case 'b': there is an obstale -> handle it
		else if(obstacle_detected_ == 1)
			change_state(1); //obstacle handler
		else
		{
			ROS_INFO("Unknown state");
		}
	}	//the yaw is not correct, so we fix it
	else if(cmath::fabs(err_yaw) > yaw_precision_)
		change_state(2); //fix_yaw
}

/*yaw_check: returns the yaw error*/
double yaw_check(geometry_msgs::Point des_pos)
{
	double desired_yaw = 0.0;
	double err_yaw     = 0.0;
	
	//where position_ is the global variable equal to the position of the robot
	desired_yaw = cmath::atan2(des_pos.y - position_.y, des_pos.x - position_.x);
	
	err_yaw = desired_yaw - yaw_;

	return err_yaw;
}

/*TBC*/
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int k = 0;	
}

void clbk_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
	
}
/*TBC*/
void fix_yaw(geometry_msgs::Point des_pos)
{
	double err_yaw = 0.0;
	
	err_yaw = yaw_check(des_pos);
	
	geometry_msgs::Twist twist_msg;

}

void fix_yaw()
{

}

/*state changing function*/
void change_state(int next_state)
{
	state_ = next_state
	ROS_INFO("State changed to [state %d]", state_);
}
/*"job done" function"*/
void done()
{
	/*publishes a Twist message that stops the robot*/
	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x  = 0;
	twist_msg.angular.z = 0;
	motion_pub.publish(twist_msg);
}
