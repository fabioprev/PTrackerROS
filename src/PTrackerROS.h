#pragma once

#include <ros/node_handle.h>

class PTracker;

class PTrackerROS
{
	private:
		ros::NodeHandle nodeHandle;
		
		PTracker* pTracker;
		
	public:
		PTrackerROS();
		
		virtual ~PTrackerROS();
};
