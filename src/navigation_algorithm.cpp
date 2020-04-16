/*
TO BE DONE: garnicht
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"

#define _USE_MATH_DEFINES

#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>

/*--functions declarations--------------------------*/
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
void clbk_odom(const nav_msgs::Odometry::ConstPtr& msg);
void fix_yaw(geometry_msgs::Point des_pos);
void change_state(int next_state);
void state_changer(geometry_msgs::Point des_pos);
void go_fwd();
double yaw_check(geometry_msgs::Point des_pos);
double pos_check(geometry_msgs::Point des_pos);
float get_min(const sensor_msgs::LaserScan::ConstPtr& msg, int range_min, int range_max);
void get_over(float sft_dist);
void done();

/*------------------------------------------------------*/
/*--global variables------------------------------------*/
/*------------------------------------------------------*/
ros::Publisher motion_pub = {};

/*pitch and roll won't be used initially*/
double roll_  = 0;
double pitch_ = 0;
double yaw_   = 0;

double yaw_precision_ = M_PI/90;
double dist_precision = 0.3;
geometry_msgs::Point position_;
geometry_msgs::Point desired_position_;

int state_ = 0;	//machine "decision" state
int obstacle_detected_ = 0;

/*obstacle handler global variables*/
float regions_[5] = {};
float sft_dist_ = 1; //safety distance
float go_fwd_  = 0.5; //linear motion
float try_fwd_ = 0.3; //linear motion when trying to move fwd
float turn_    = 0.3; //angular motion when obstacle is found (0.3 is turning right)

/*------------------------------------------------------*/
/*--main function---------------------------------------*/
/*------------------------------------------------------*/
int main(int argc, char **argv)
{
	/*initializing the various topic subscriptions and publishes*/
	ros::init(argc, argv, "reading_laser");
	
	ros::NodeHandle n;
	
	::motion_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	ros::Subscriber motion_sub = n.subscribe("/m2wr/laser/scan", 100, clbk_laser);
	
	ros::Subscriber odom_sub = n.subscribe("/odom", 100, clbk_odom);

	desired_position_.x = -3;
	desired_position_.y = -8;
	desired_position_.z = 0;
	
	while (ros::ok()){
		/*finite state machine*/
		state_changer(desired_position_);
	
		if(state_ == 0)
		{
			go_fwd();
		}
		else if(state_ == 1)
		{
			fix_yaw(desired_position_);//**/
		}
		else if(state_ == 2)
		{
			get_over(sft_dist_);//*/
		}
		else if(state_ == 3)
		{
			done();
			return 0;
		}
		else
		{
			ROS_INFO("Unknown state!");
			return 0;
		}
	
		ros::spinOnce();
	}
	return 0;
	
}
/*------------------------------------------------------*/
/*--functions definitions-------------------------------*/
/*------------------------------------------------------*/

/*--obstacle handler ---------------------------------------------*/
void get_over(float sft_dist)
{
	
	geometry_msgs::Twist msg;
	msg = {};
	
	double linear_x  = 0.0;
	double angular_z = 0.0;
	
	std_msgs::String state_description;
	
	/*	
	 *	 we will exclude from computations the border regions left and right!
	 *
	 *    |----------------/front\---------------|
	 *    |--------/fright/.......\fleft\--------|
	 *    |-left--/......................\-right-|
	 *
	 */
	float fright = regions_[1];
	float front  = regions_[2];
	float fleft  = regions_[3];
	
	
	ROS_INFO("Handling obstacle");
	
	fix_yaw(desired_position_);
	
	if((fright > sft_dist) && (front > sft_dist) && (fleft > sft_dist))
	{
		state_description.data = "case 1 - nothing [go straight]";
		linear_x = go_fwd_;
		angular_z = 0;
	}
	else if((fright > sft_dist) && (front > sft_dist) && (fleft < sft_dist))
	{
		state_description.data = "case 2 - fleft [turn right]";
		linear_x = 0;
		angular_z = turn_;
	}
	else if((fright > sft_dist) && (front < sft_dist) && (fleft > sft_dist))
	{
		state_description.data = "case 3 - front [turn right]";
		linear_x = 0;
		angular_z = turn_;
		}
	else if((fright > sft_dist) && (front < sft_dist) && (fleft < sft_dist))
	{
		state_description.data = "case 4 - front and fleft [turn right]";
		linear_x = 0;
		angular_z = turn_;
	}
	else if((fright < sft_dist) && (front > sft_dist) && (fleft > sft_dist))
	{
		state_description.data = "case 5 - fright [turn left]";
		linear_x = 0;
		angular_z = -turn_;
	}
	else if((fright < sft_dist) && (front > sft_dist) && (fleft < sft_dist))
	{
		state_description.data = "case 6 - fright and fleft [try straight]";
		linear_x = try_fwd_;
		angular_z = 0;
	}
	else if((fright < sft_dist) && (front < sft_dist) && (fleft > sft_dist))
	{
		state_description.data = "case 7 - fright and front [turn left]";
		linear_x = 0;
		angular_z = -turn_;
	}
	else if((fright < sft_dist) && (front < sft_dist) && (fleft < sft_dist))	
	{
		state_description.data = "case 8 - fright and front and fleft [turn right]";
		linear_x = 0;
		angular_z = turn_;
	}
	else
	{
		state_description.data = "unknown case";
		ROS_INFO("%f-%f-%f-%f-%f", regions_[0], regions_[1], regions_[2], regions_[3], regions_[4]);
		
	}

	ROS_INFO("[%s]", state_description.data.c_str());
	
	msg.linear.x  = linear_x ; 
	msg.angular.z = angular_z;
		
	motion_pub.publish(msg);	
}

/*--state_changer: determines the current machine state-----------*/
void state_changer(geometry_msgs::Point des_pos)
{
	double err_yaw = 0.0;
	double err_pos = 0.0;
	
	err_yaw = yaw_check(des_pos);
	err_pos = pos_check(des_pos);
	
	//if we arrived that the objective we finish the mission
	if(err_pos <= dist_precision)
		change_state(3);
	
	//check if the yaw is correct, if not, we fix it
	if(fabs(err_yaw) <= yaw_precision_)
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
	else if(fabs(err_yaw) > yaw_precision_)
		change_state(2); //fix_yaw
}

/*--yaw_check: returns the yaw error------------------------------*/
double yaw_check(geometry_msgs::Point des_pos)
{
	double desired_yaw = 0.0;
	double err_yaw     = 0.0;
	
	//where position_ is the global variable equal to the position of the robot
	desired_yaw = atan2(des_pos.y - position_.y, des_pos.x - position_.x);
	
	err_yaw = desired_yaw - yaw_;

	return err_yaw;
}

double pos_check(geometry_msgs::Point des_pos)
{
	double err_pos = 0.0;
	
	err_pos = sqrt(pow(des_pos.y - position_.y, 2.0) + pow(des_pos.x - position_.x, 2.0));
	
	return err_pos;
}

/*--go straight forward function----------------------------------*/
/*	publishes a simple "go straight" geometry message 						*/
void go_fwd()
{
	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x = 0.3;
	motion_pub.publish(twist_msg);
}

/*--laser callback function---------------------------------------*/
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int k = 0;
	
	//right = min_regions[0], fright = min_regions[1], front = min_regions[2], ...
	
	//calculates the minimum for each of the 5 regions
	for(k = 0; k < 5; k++){
		regions_[k] = get_min(msg, k*144, (k+1)*144);
	}
	//now we have the position of the closes object for each region in the min_regions[] array
}

/*--get minimum object function-----------------------------------*/
float get_min(const sensor_msgs::LaserScan::ConstPtr& msg, int range_min, int range_max){
	
	int k = 0;
	float temp_min = 0;
	/*assigns the starting value to temp_min*/
	temp_min = msg->ranges[range_min];
	
	/*simple find_minimum function but with the ranges as parameters (range_min, range_max)*/
	for(k = range_min; k < range_max; k++)
	{
		if(msg->ranges[k] < temp_min)
			temp_min = msg->ranges[k];
	}
	
	/*if the min value exceeds 10 meters, it would return 'inf', which we cut to 10*/
	if(temp_min > 10)
		temp_min = 10;

	return temp_min;
	
}


/*--odometry callback function -----------------------------------*/
void clbk_odom(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	position_ = odom_msg->pose.pose.position;
	
	tf::Quaternion q(odom_msg->pose.pose.orientation.x,
									 odom_msg->pose.pose.orientation.y,
									 odom_msg->pose.pose.orientation.z,
									 odom_msg->pose.pose.orientation.w);
	
	tf::Matrix3x3 m(q);
	
	m.getRPY(roll_, pitch_, yaw_);
}

/*--fix yaw function----------------------------------------------*/
void fix_yaw(geometry_msgs::Point des_pos)
{
	double err_yaw = 0.0;
	//gets the yaw error from the correct orientation
	err_yaw = yaw_check(des_pos);
	
	geometry_msgs::Twist twist_msg;
	
	if(fabs(err_yaw) > yaw_precision_)
	{
		if(err_yaw > 0)
		  //if the yaw error is positive we need to turn right, and viceversa
			twist_msg.angular.z = -0.3;
		else if(err_yaw < 0)
			twist_msg.angular.z =  0.3;
	}
	
	motion_pub.publish(twist_msg);

}

/*--state changing function---------------------------------------*/
void change_state(int next_state)
{
	state_ = next_state;
	ROS_INFO("State changed to [state %d]", state_);
}

/*--"job done" function"------------------------------------------*/
void done()
{
	/*publishes a Twist message that stops the robot*/
	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x  = 0;
	twist_msg.angular.z = 0;
	motion_pub.publish(twist_msg);
}
