#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <iostream>

ros::Publisher motion_pub = {};

/*functions declarations*/
float get_min(const sensor_msgs::LaserScan::ConstPtr& msg, int range_min, int range_max);
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
void take_actions(float regions[], float sft_dist);

/*global variables*/
float sft_dist_ = 1.0; //safety distance

int main(int argc, char **argv)
{

	ros::init(argc, argv, "reading_laser");
	
	ros::NodeHandle n;
	
	::motion_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	ros::Subscriber motion_sub = n.subscribe("/m2wr/laser/scan", 100, clbk_laser);
	
	ros::spin();
	
	return 0;
}

//callback function
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int k = 0;
	
	//right = min_regions[0], frr = min_regions[1], ffr = min_regions[2], ...
	float min_regions[5] = {};
	
	//calculates the minimum for each of the 6 regions
	for(k = 0; k < 5; k++){
		min_regions[k] = get_min(msg, k*144, (k+1)*144);
	}
	//now we have the position of the closes object for each region in the min_regions[] array
	
	take_actions(min_regions, sft_dist_);
	
}

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

/*	function description: take_actions
 *
 * 	param@ regions[]: array containing the position of closest object for each region
 *	param@ sft_dist: safety distance (distance at which we avoid an obstacle)	
 *
 *	returns@ void
 *
 *	brief@ : avoids obstacles that are closer than the safety distance
 */
 
void take_actions(float regions[], float sft_dist)
{
	
	geometry_msgs::Twist msg;
	msg = {};
	
	double linear_x  = 0.0;
	double angular_z = 0.0;
	
	std_msgs::String state_description;
	
	/*	
	 *	 we will exclude from computations the border regions left and right!
	 *
	 *    |------------------/front\--------------|
	 *    |--------/fright--/      \--fleft\------|
	 *    |-right-/                        \-left-|
	 *
	 */
	float fright = regions[1];
	float front  = regions[2];
	float fleft  = regions[3];
	
	if((fright > sft_dist) && (front > sft_dist) && (fleft > sft_dist))
	{
		state_description.data = "case 1 - nothing [go straight]";
		linear_x = 0.5;
		angular_z = 0;
	}
	else if((fright > sft_dist) && (front > sft_dist) && (fleft < sft_dist))
	{
		state_description.data = "case 2 - fleft [turn right]";
		linear_x = 0;
		angular_z = 0.3;
	}
	else if((fright > sft_dist) && (front < sft_dist) && (fleft > sft_dist))
	{
		state_description.data = "case 3 - front [turn right]";
		linear_x = 0;
		angular_z = 0.3;
		}
	else if((fright > sft_dist) && (front < sft_dist) && (fleft < sft_dist))
	{
		state_description.data = "case 4 - front and fleft [turn right]";
		linear_x = 0;
		angular_z = 0.3;
	}
	else if((fright < sft_dist) && (front > sft_dist) && (fleft > sft_dist))
	{
		state_description.data = "case 5 - fright [turn left]";
		linear_x = 0;
		angular_z = -0.3;
	}
	else if((fright < sft_dist) && (front > sft_dist) && (fleft < sft_dist))
	{
		state_description.data = "case 6 - fright and fleft [try straight]";
		linear_x = 0.3;
		angular_z = 0;
	}
	else if((fright < sft_dist) && (front < sft_dist) && (fleft > sft_dist))
	{
		state_description.data = "case 7 - fright and front [turn left]";
		linear_x = 0;
		angular_z = -0.3;
	}
	else if((fright < sft_dist) && (front < sft_dist) && (fleft < sft_dist))	
	{
		state_description.data = "case 8 - fright and front and fleft [turn right]";
		linear_x = 0;
		angular_z = 0.3;
	}
	else
	{
		state_description.data = "unknown case";
		ROS_INFO("%f-%f-%f-%f-%f", regions[0], regions[1], regions[2], regions[3], regions[4]);
		
	}

	ROS_INFO("[%s]", state_description.data.c_str());
	
	msg.linear.x  = linear_x ; 
	msg.angular.z = angular_z;
		
	motion_pub.publish(msg);	
}
// end of take_action


