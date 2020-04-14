#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include <iostream>


float get_min(const sensor_msgs::LaserScan::ConstPtr& msg, int range_min, int range_max){
	
	
	int k = 0;
	float temp_min = 0;
	
	temp_min = msg->ranges[range_min];
	
	for(k = range_min; k < range_max; k++)
	{
		if(msg->ranges[k] < temp_min)
			temp_min = msg->ranges[k];
	}
	
	if(temp_min > 10)
		temp_min = 10;

	return temp_min;
	
}


void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int k = 0;
	
	float min_regions[6] = {};
	
	//calculates the minimum for each of the 6 regions
	for(k = 0; k < 6; k++){
		min_regions[k] = get_min(msg, k*120, (k+1)*120);
	}
	
  ROS_INFO("\nright: [%f]\nfrr:   [%f]\nffr:   [%f]\nffl:   [%f]\nfll:   [%f]\nleft:  [%f]", min_regions[0], min_regions[1], min_regions[2], min_regions[3], min_regions[4], min_regions[5]  	
  );

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "reading_laser");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/m2wr/laser/scan", 100, clbk_laser);

  ros::spin();

  return 0;
}

/* FUNCTIONS DEFINITIONS */



