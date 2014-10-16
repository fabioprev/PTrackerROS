#pragma once

#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <Utils/Point2of.h>

class PTracker;

class PTrackerROS
{
	private:
		std::map<std::string,PTracking::Point2of> robotPoses;
		std::vector<ros::Subscriber> subscriberRobotPoses;
		ros::NodeHandle nodeHandle;
		PTracker* pTracker;
		boost::mutex mutex;
		double maxReading;
		int agentId;
		
	public:
		PTrackerROS();
		
		virtual ~PTrackerROS();
		
		void exec();
		void updateRobotPose(const nav_msgs::Odometry::ConstPtr& message);
};
