#include "PTrackerROS.h"
#include <ros/ros.h>

int main (int argc, char** argv)
{
	ros::init(argc,argv,"PTrackerROS");
	
	PTrackerROS pTrackerROS;
	
	while (ros::ok())
	{
		ros::spinOnce();
		
		pTrackerROS.exec();
		
		usleep(5e3);
	}
	
	return 0;
}
