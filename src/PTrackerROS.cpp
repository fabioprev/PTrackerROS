#include "PTrackerROS.h"
#include <PTracker/PTracker.h>

PTrackerROS::PTrackerROS() : nodeHandle("~"), pTracker(0)
{
	int agentId;
	
	nodeHandle.getParam("agentId",agentId);
	
	pTracker = new PTracker(agentId,string(getenv("PTracking_ROOT")) + string("/../config/parameters.cfg"));
}

PTrackerROS::~PTrackerROS()
{
	if (pTracker != 0) delete pTracker;
}
