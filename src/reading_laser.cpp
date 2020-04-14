#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"

typedef struct {
	int min;
	int max;
}	region;

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
		region front;
		front.min = 0;
		front.max = 143; 
		
		ROS_INFO("");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reading_laser");
    
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("/m2wr/laser/scan", 1000, clbk_laser);
    
    ros::spin();
    return 0;
}
