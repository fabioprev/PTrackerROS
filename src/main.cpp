#include "PTrackerROS.h"
#include <ros/ros.h>

int main (int argc, char** argv)
{
	ros::init(argc,argv,"PTrackerROS");
	
	PTrackerROS pTrackerROS;
	
	ros::spin();
	
	return 0;
}
