#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <cmath>
#include <sstream>
#include <string>
#include "tf/tf.h"
#define _USE_MATH_DEFINES


/*--global variables----------------------------------------------*/
ros::Publisher motion_pub_ = {};
/*right = min_regions[0], fright = min_regions[1], front = min_regions[2], ... */
float min_regions_[5] = {};
float sft_dist_ = 1.5; 

geometry_msgs::Point position_;
double roll_  = 0.0;
double pitch_ = 0.0;
double yaw_   = 0.0;

geometry_msgs::Point desired_position_;

double yaw_precision_ = M_PI/90;
double dist_precision_ = 0.3;


int state_ = 0;

/*--functions declarations----------------------------------------*/
void clbk_odom(const nav_msgs::Odometry::ConstPtr& msg);
void go_straight_ahead(geometry_msgs::Point des_pos);
void change_state(int state);
void done(void);
void fix_yaw(geometry_msgs::Point des_pos);

/*--main----------------------------------------------------------*/
int main(int argc, char **argv)
{

	desired_position_.x = -3;
	desired_position_.y =  7;
	desired_position_.z =  0;

	ros::init(argc, argv, "go_to_point");
		
	ros::NodeHandle n;
		
	ros::Subscriber motion_sub;
		
	motion_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		
	motion_sub = n.subscribe("/odom", 1, clbk_odom);
		
				
	while(ros::ok())
	{
		if(state_ == 0)
			fix_yaw(desired_position_);
		else if(state_ == 1)
			go_straight_ahead(desired_position_);
		else if(state_ == 2)
			done();
		else
		{
			ROS_INFO("Unknown state!");
		}
		
		ros::spinOnce();
	}
		
	return 0;
}

/*--functions definitions-----------------------------------------*/
void clbk_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
	position_ = msg->pose.pose.position;
  /*get the yaw value*/
  tf::Quaternion q(msg->pose.pose.orientation.x,
                   msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z,
                   msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);
}

void go_straight_ahead(geometry_msgs::Point des_pos)
{
	double desired_yaw, err_yaw, err_pos;
	
	desired_yaw = atan2(des_pos.y - position_.y, des_pos.x - position_.x);
	err_yaw = desired_yaw - yaw_;
	err_pos = sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2));
	
	/*we first check that the robot isn't already in position*/
	if(err_pos > dist_precision_)
	{
		/*we are not yet at objective, so we go straight ahead*/
		geometry_msgs::Twist twist_msg = {};
		twist_msg.linear.x = 0.3;
	  motion_pub_.publish(twist_msg);
	}
	else	
	{
		/*we have arrived*/
		ROS_INFO("Yaw error: [%f]", err_yaw);
		change_state(2);
	}
	
	/*if the yaw error exceeds the correct value, we fix it*/
	if(fabs(err_yaw) > yaw_precision_)
	{
		ROS_INFO("Yaw error: [%f]", err_yaw);
		change_state(0);
	}
}

void change_state(int state)
{
	state_ = state;
	ROS_INFO("State changed to [%d]", state_);
}

void done(void)
{
	geometry_msgs::Twist twist_msg = {};
	twist_msg.linear.x  = 0;
	twist_msg.angular.z = 0;
	motion_pub_.publish(twist_msg);
}

void fix_yaw(geometry_msgs::Point des_pos)
{
	double desired_yaw, err_yaw;
	
	desired_yaw = atan2(des_pos.y - position_.y, des_pos.x - position_.x);
	err_yaw = desired_yaw - yaw_;
	
	geometry_msgs::Twist twist_msg;
	
	if(fabs(err_yaw) > yaw_precision_)
	{
		if(err_yaw > 0)
			twist_msg.angular.z = -0.3;
		else
			twist_msg.angular.z =  0.3;
	}
	motion_pub_.publish(twist_msg);
	
	if(fabs(err_yaw) <= yaw_precision_)
	{
		ROS_INFO("Yaw error: [%f]", err_yaw);
		change_state(1);
	}
}





























