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
float sft_dist_ = 1.0;

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
	float min_regions[6] = {};
	
	//calculates the minimum for each of the 6 regions
	for(k = 0; k < 6; k++){
		min_regions[k] = get_min(msg, k*120, (k+1)*120);
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
 *	param@ num_r: number of total regions (also length of the min_regions[] array)
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
	
	std::string state_description = "";
	
	/*	regions:| 	0		|		1		|		2		|		3		|		4		|		5 	|
	 *					| right	|  frr  |	 ffr	|	 ffl 	|	 fll	|	left	|
	 *	but we will exclude from computations the border regions left and right!
	 */
	float frr   = regions[1];
	float ffr		= regions[2];
	float ffl		= regions[3];
	float fll		= regions[4];
	
	while(ros::ok())
	{
		if((frr > sft_dist) && (ffr > sft_dist) && (ffl > sft_dist) && (fll > sft_dist))
		{
			state_description = "case 1 - nothing";
			linear_x = 0.6;
			angular_z = 0;
		}
		else if((frr > sft_dist) && (ffr > sft_dist) && (ffl > sft_dist) && (fll < sft_dist))
		{
			state_description = "case 2 - fll";
			linear_x = 0;
			angular_z = 0.3;
		}
		else if((frr > sft_dist) && (ffr > sft_dist) && (ffl < sft_dist) && (fll > sft_dist))
		{
			state_description = "case 3 - ffl";
			linear_x = 0;
			angular_z = 0.3;
			}
		else if((frr > sft_dist) && (ffr > sft_dist) && (ffl < sft_dist) && (fll < sft_dist))
		{
			state_description = "case 4 - ffl and fll";
			linear_x = 0;
			angular_z = 0.3;
		}
		else if((frr > sft_dist) && (ffr < sft_dist) && (ffl > sft_dist) && (fll > sft_dist))
		{
			state_description = "case 5 - ffr";
			linear_x = 0;
			angular_z = -0.3;
		}
		else if((frr > sft_dist) && (ffr < sft_dist) && (ffl > sft_dist) && (fll < sft_dist))
		{
			state_description = "case 6 - ffr and fll";
			linear_x = 0;
			angular_z = 0.3;
		}
		else if((frr > sft_dist) && (ffr < sft_dist) && (ffl < sft_dist) && (fll > sft_dist))
		{
			state_description = "case 7 - ffr and ffl";
			linear_x = 0;
			angular_z = 0.3;
		}
		else if((frr > sft_dist) && (ffr < sft_dist) && (ffl < sft_dist) && (fll < sft_dist))
		{
			state_description = "case 8 - ffr and ffl and fll";
			linear_x = 0;
			angular_z = 0.3;
		}
		else if((frr < sft_dist) && (ffr > sft_dist) && (ffl > sft_dist) && (fll > sft_dist))
		{
			state_description = "case 9 - frr";
			linear_x = 0;
			angular_z = -0.3;
		}
		else if((frr < sft_dist) && (ffr > sft_dist) && (ffl > sft_dist) && (fll < sft_dist))
		{
			state_description = "case 10 - frr and fll";
			linear_x = 0.3;
			angular_z = 0;
		}
		else if((frr < sft_dist) && (ffr > sft_dist) && (ffl < sft_dist) && (fll > sft_dist))
		{
			state_description = "case 11 - frr and ffl";
			linear_x = 0;
			angular_z = -0.3;//
		}
		else if((frr < sft_dist) && (ffr > sft_dist) && (ffl < sft_dist) && (fll < sft_dist))
		{
			state_description = "case 12 - frr and ffl and fll";
			linear_x = 0;
			angular_z = 0.3;//
	
		}
		else if((frr < sft_dist) && (ffr < sft_dist) && (ffl > sft_dist) && (fll > sft_dist))
		{
			state_description = "case 13 - frr and ffr";
			linear_x = 0;
			angular_z = -0.3;//
		}
		else if((frr < sft_dist) && (ffr < sft_dist) && (ffl > sft_dist) && (fll < sft_dist))
		{
			state_description = "case 14 - frr and ffr and fll";
			linear_x = 0;
			angular_z = -0.3;//
		}
		else if((frr < sft_dist) && (ffr < sft_dist) && (ffl < sft_dist) && (fll > sft_dist))
		{
			state_description = "case 15 - frr and ffr and ffl";
			linear_x = 0;
			angular_z = 0.3;
		}
		else if((frr < sft_dist) && (ffr < sft_dist) && (ffl < sft_dist) && (fll < sft_dist))
		{
			state_description = "case 16 - frr and ffr and ffl and fll";
			linear_x = 0;
			angular_z = 0.3;
		}
		else
		{
			state_description = "unknown case";
			ROS_INFO("%f-%f-%f-%f-%f-%f", regions[0], regions[1], regions[2], regions[3], regions[4], regions[5]);
			
		}
		
		std::stringstream ss;
		
		ss << state_description ;
		
		msg.linear.x  = linear_x ; 
		msg.angular.z = angular_z;
		
		motion_pub.publish(msg);
	}
}
// end of take_action


