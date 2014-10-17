#pragma once

#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <LaserScanDetector/ObjectDetection.h>
#include <Utils/Utils.h>

class PTracker;

class PTrackerROS
{
	private:
		std::vector<LaserScanDetector::Object> objectDetected;
		ros::NodeHandle nodeHandle;
		ros::Subscriber subscriberObjectDetected;
		boost::mutex mutex, mutexDetection;
		PTracking::Point2of robotPose;
		PTracker* pTracker;
		int agentId;
		
	public:
		PTrackerROS();
		
		virtual ~PTrackerROS();
		
		void exec();
		void updateObjectDetected(const LaserScanDetector::ObjectDetection::ConstPtr& message);
		void updateRobotPose(const nav_msgs::Odometry::ConstPtr& message);
};
