#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include <iostream>


/*	function description: get_min
 *
 * 	param@ msg : 
 *	param@ range_min: represents the first value of the considered region (0 := right)
 *	param@ range_max: represente the last value of the considered region (719:= last one)
 *
 *	returns@ the minimum value found in the msg->ranges[]
 */
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


void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int k = 0;
	
	float min_regions[5] = {};
	
	//calculates the minimum for each of the 6 regions
	for(k = 0; k < 5; k++){
		min_regions[k] = get_min(msg, k*144, (k+1)*144);
	}
	
  ROS_INFO("\nright: [%f]\nfright:   [%f]\nfront:   [%f]\nfleft:   [%f]\nleft:  [%f]", min_regions[0], min_regions[1], min_regions[2], min_regions[3], min_regions[4]);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "reading_laser");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/m2wr/laser/scan", 100, clbk_laser);

  ros::spin();

  return 0;
}



