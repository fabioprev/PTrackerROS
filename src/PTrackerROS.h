#pragma once

#include <ros/node_handle.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <LaserScanDetector/ObjectDetection.h>
#include <Utils/Utils.h>

/// Uncomment if you want get the robot pose from the localizer algorithm.
//#define LOCALIZER

class PTracker;

class PTrackerROS
{
	private:
		std::vector<LaserScanDetector::Object> objectDetected;
		ros::NodeHandle nodeHandle;
		ros::Subscriber subscriberObjectDetected, subscriberRobotPose;
		boost::mutex mutex, mutexDetection;
		PTracking::Point2of robotPose;
		PTracker* pTracker;
		int agentId;
		
	public:
		PTrackerROS();
		
		virtual ~PTrackerROS();
		
		void exec();
		void updateObjectDetected(const LaserScanDetector::ObjectDetection::ConstPtr& message);
		
#ifndef LOCALIZER
		void updateRobotPose(const nav_msgs::Odometry::ConstPtr& message);
#else
		void updateRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message);
#endif
};
