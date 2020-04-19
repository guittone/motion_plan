/*
 __  __   ______   __  __   ______   ____     ______   ______  ______   _____    __  __     
/\ \/\ \ /\  _  \ /\ \/\ \ /\__  _\ /\  _`\  /\  _  \ /\__  _\/\__  _\ /\  __`\ /\ \/\ \    
\ \ `\\ \\ \ \L\ \\ \ \ \ \\/_/\ \/ \ \ \L\_\\ \ \L\ \\/_/\ \/\/_/\ \/ \ \ \/\ \\ \ `\\ \   
 \ \ , ` \\ \  __ \\ \ \ \ \  \ \ \  \ \ \L_L \ \  __ \  \ \ \   \ \ \  \ \ \ \ \\ \ , ` \  
  \ \ \`\ \\ \ \/\ \\ \ \_/ \  \_\ \__\ \ \/, \\ \ \/\ \  \ \ \   \_\ \__\ \ \_\ \\ \ \`\ \ 
   \ \_\ \_\\ \_\ \_\\ `\___/  /\_____\\ \____/ \ \_\ \_\  \ \_\  /\_____\\ \_____\\ \_\ \_\
    \/_/\/_/ \/_/\/_/ `\/__/   \/_____/ \/___/   \/_/\/_/   \/_/  \/_____/ \/_____/ \/_/\/_/

**********************************************
*-Navigation algorithm for a 2 wheeled robot-*
**********************************************
by Guerrini Daniele and Scaloni Alessandro.

PROGRAM FLOW:

******		************		****************
*MAIN* -> *CLBK_LASER* -> *DECIDE_ACTIONS* 
******		************		****************
  |												 
  V
***********					
*CLBK_ODOM*
***********

*/                                                                                          

/* -- INCLUDE AND DEFINES --------------------------------------- */
/*ros libraries*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h" 
#include "tf/transform_listener.h" 
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
/*c++ libraries*/
#include <cmath>

#define _USE_MATH_DEFINES
/* -- GLOBAL VARIABLES ------------------------------------------ */
ros::Publisher motion_pub_ = {};

typedef enum 
{
		TURN_LEFT = 0,     /*we need to turn left*/
		TURN_RIGHT,				 /*we need to turn right*/
		GO_TO_POINT,       /*we need to go to point*/
		FOLLOW_WALL_RIGHT, /*we need to follow right*/
		FOLLOW_WALL_LEFT,  /*we need to follow left*/
		UNKNOWN            /*unknown case*/
}	ROBOT_STATE;

ROBOT_STATE state_;

typedef enum
{
	FIX_YAW = 0, /*we need to fix heading*/
	STRAIGHT,    /*we need to go straight*/
	ARRIVED      /*we have arrived*/
}	GOING_TO_POINT_STATE;

GOING_TO_POINT_STATE gtp_state_ = FIX_YAW; /*go to point global variable*/

float min_regions_[5] = {};

double roll_ = 0.0;
double pitch_ = 0.0;
double yaw_ = 0.0;
double err_yaw_;
double err_pos_ = 1000;
double yaw_precision_ = M_PI/90;
double dist_precision_ = 0.3;
float sft_dist_ = 1.0;
/*angular.z at which the robot turns when turning left*/
float TURN_LEFT_ANGULAR_ = -0.6;
/*angular.z at which the robot turns when turning right*/
float TURN_RIGHT_ANGULAR_ =  0.6;
/*linear.x at which the robot goes straight*/
float GO_STRAIGHT_LINEAR_ = 0.5;
/*linear.x at which the robot moves fwd while fixing yaw*/
float FIX_YAW_LINEAR_ = 0.3;
/*linear.x at which the robot moves while following wall*/
float FOLLOW_WALL_LINEAR_ = 0.5;

geometry_msgs::Point position_;
geometry_msgs::Point desired_position_;
/* ************************************************************** */
/* -- FUNCTIONS DECLARATIONS ------------------------------------ */
/*1*/void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
/*2*/float get_min(const sensor_msgs::LaserScan::ConstPtr& msg, int index1, int index2, float max_dist);
/*3*/void decide_actions(float min_regions_[], float sft_dist);
/*4*/void robot_moving(ROBOT_STATE move_type);
/*5*/void go_straight(const geometry_msgs::Point des_pos, geometry_msgs::Twist twist_msg);
/*6*/void fix_yaw(const geometry_msgs::Point des_pos, geometry_msgs::Twist twist_msg);
/*7*/void clbk_odom(const nav_msgs::Odometry::ConstPtr& odom_msg);
/* ************************************************************** */
/* -- FUNCTIONS DEFINITIONS ------------------------------------- */
/*0* main */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigation_node");
	
	ros::NodeHandle n;
	
	ros::Subscriber laser_sub, odom_sub;
	
	odom_sub = n.subscribe("/odom", 100, clbk_odom);
	
	laser_sub = n.subscribe("m2wr/laser/scan", 100, clbk_laser);
	
	motion_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	desired_position_.x =  8;
	desired_position_.y = -8;
	desired_position_.z =  0;
	
	while(ros::ok())
	{
		ros::spinOnce();
	}
	
	return 0;
}
/*1* clbk_laser */
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int k;
	/*laser points per region*/
	int lsr_p = 144; 
	float max_dist = 10.0;
	
	for(k = 0; k < 6; k++)
	{
		min_regions_[k] = get_min(msg, lsr_p*k, lsr_p*(k+1), max_dist);
	}
	
	decide_actions(min_regions_, sft_dist_);
}
/*end1*/

/*2* get_min */
float get_min(const sensor_msgs::LaserScan::ConstPtr& msg, int index1, int index2, float max_dist)
{
	int i;
	float temp_min = msg->ranges[index1];
	
	for(i = index1; i < index2; i++)
	{
		if(msg->ranges[i] < temp_min)
			{temp_min = msg->ranges[i];}
		if(temp_min > max_dist)
			{temp_min = max_dist;}
	}	
	return temp_min;
}
/*end2*/

/*3* decide_actions */
void decide_actions(float min_regions[], float sft_dist)
{
	float right  = min_regions[0];
	float fright = min_regions[1];
	float front  = min_regions[2];
	float fleft  = min_regions[3];
	float left   = min_regions[4];
/*typedef enum
{
	FIX_YAW = 0, 
	STRAIGHT,    
	ARRIVED      
}	GOING_TO_POINT_STATE;*/

	/*first condition checks if we haven't arrived yet*/
	if(gtp_state_ != ARRIVED)
	{
		if(     (fright > sft_dist_) && (front > sft_dist_) && (fleft > sft_dist_))
		{
			state_ = GO_TO_POINT;
		}
		else if((fright > sft_dist_) && (front > sft_dist_) && (fleft < sft_dist_))
		{
			state_ = FOLLOW_WALL_LEFT;
		}
		else if((fright > sft_dist_) && (front < sft_dist_) && (fleft > sft_dist_))
		{
			(err_yaw_ > 0) ? (state_ = TURN_RIGHT) : (state_ = TURN_LEFT);
		}
		else if((fright > sft_dist_) && (front < sft_dist_) && (fleft < sft_dist_))
		{
			(err_yaw_ > 0) ? (state_ = TURN_RIGHT) : (state_ = TURN_LEFT);
		}
		else if((fright < sft_dist_) && (front > sft_dist_) && (fleft > sft_dist_))
		{
			state_ = FOLLOW_WALL_RIGHT;
		}
		else if((fright < sft_dist_) && (front > sft_dist_) && (fleft < sft_dist_))
		{
			(err_yaw_ > 0) ? (state_ = TURN_RIGHT) : (state_ = TURN_LEFT);
		}
		else if((fright < sft_dist_) && (front < sft_dist_) && (fleft > sft_dist_))
		{
			(err_yaw_ > 0) ? (state_ = TURN_RIGHT) : (state_ = TURN_LEFT);
		}
		else if((fright < sft_dist_) && (front < sft_dist_) && (fleft < sft_dist_))
		{
			(err_yaw_ > 0) ? (state_ = TURN_RIGHT) : (state_ = TURN_LEFT);
		}
		else
		{
			state_ = UNKNOWN;
			ROS_INFO("%f-%f-%f-%f-%f [UNKNOWN CASE]", min_regions[0], min_regions[1], min_regions[2], min_regions[3], min_regions[4]);
		}
		
		robot_moving(state_);
	}
	else if(err_pos_ < dist_precision_)/*if we are at destination*/
	{
		ROS_INFO("Mission completed!");
		
		geometry_msgs::Twist twist_msg;
		twist_msg.linear.x = 0.0;
		twist_msg.angular.z = 0.0;
		
		motion_pub_.publish(twist_msg);	
	}
}
/*end3*/

/*4* robot_moving*/
void robot_moving(ROBOT_STATE move_type)
{
	geometry_msgs::Twist twist_msg = {};	
	
	if(err_pos_ < dist_precision_)
	{
		gtp_state_ = ARRIVED;
	}
	
	if(move_type == GO_TO_POINT)
	{
		ROS_INFO("Going to the point");
		
		if(gtp_state_ == FIX_YAW)
		{
			fix_yaw(desired_position_, twist_msg);
		}
		else if(gtp_state_ == STRAIGHT)
		{
			go_straight(desired_position_, twist_msg);
		}
	}
	else if(move_type == TURN_LEFT)
	{
		ROS_INFO("Turning right!");
		twist_msg.linear.x  = 0.0;
		twist_msg.angular.z = TURN_LEFT_ANGULAR_;
		motion_pub_.publish(twist_msg);
	}
	else if(move_type == TURN_RIGHT)
	{
		ROS_INFO("Turning left!");
		twist_msg.linear.x  = 0.0;
		twist_msg.angular.z = TURN_RIGHT_ANGULAR_;
		motion_pub_.publish(twist_msg);
	}
	else if(move_type == FOLLOW_WALL_LEFT)
	{
		ROS_INFO("Avoiding the obstacle on the left!");
		twist_msg.linear.x = FOLLOW_WALL_LINEAR_;
		twist_msg.angular.z = TURN_RIGHT_ANGULAR_;
		motion_pub_.publish(twist_msg);
	}
	else if(move_type == FOLLOW_WALL_RIGHT)
	{
		ROS_INFO("Avoiding the obstacle on the right!");
		twist_msg.linear.x = FOLLOW_WALL_LINEAR_;
		twist_msg.angular.z = TURN_LEFT_ANGULAR_;
		motion_pub_.publish(twist_msg);
	}
}
/*end4*/

/*5* go_straight */
void go_straight(const geometry_msgs::Point des_pos, geometry_msgs::Twist twist_msg)
{
	double desired_yaw;
	desired_yaw = atan2(des_pos.y - position_.y, des_pos.x - position_.x);
	err_yaw_ = desired_yaw - yaw_;
	err_pos_ = sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2));
	
	if(err_pos_ > dist_precision_)
	{
		twist_msg.linear.x = GO_STRAIGHT_LINEAR_;
		twist_msg.angular.z = 0.0;
		motion_pub_.publish(twist_msg);
	}
	else
		gtp_state_ = ARRIVED;
		
	if(fabs(err_yaw_) > yaw_precision_)
	{
		gtp_state_ = FIX_YAW;
	}
}
/*end5*/

/*6* fix_yaw */
void fix_yaw(const geometry_msgs::Point des_pos, geometry_msgs::Twist twist_msg)
{
	double desired_yaw;
	desired_yaw = atan2(des_pos.y - position_.y, des_pos.x - position_.x);
	err_yaw_ = desired_yaw - yaw_;
	
	if(fabs(err_yaw_) > yaw_precision_)
	{
		if(err_yaw_ > 0)
		{
			twist_msg.linear.x = FIX_YAW_LINEAR_;
			twist_msg.angular.z = TURN_LEFT_ANGULAR_;
			motion_pub_.publish(twist_msg);
		}
		else
		{
			twist_msg.linear.x = FIX_YAW_LINEAR_;
			twist_msg.angular.z = TURN_RIGHT_ANGULAR_;
			motion_pub_.publish(twist_msg);
		}
	}
	else
	{
		gtp_state_ = STRAIGHT;
	}

}
/*end6*/

/*7* clbk_odom*/
void clbk_odom(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	     //Aggiornare la posizione del robot
     position_ = odom_msg->pose.pose.position;
     //aggiornare l'angolo del robot (yaw)
     tf::Quaternion q(odom_msg->pose.pose.orientation.x,
                      odom_msg->pose.pose.orientation.y,
                      odom_msg->pose.pose.orientation.z,
                      odom_msg->pose.pose.orientation.w);
     tf::Matrix3x3 m(q);
     m.getRPY(roll_, pitch_, yaw_);
}
