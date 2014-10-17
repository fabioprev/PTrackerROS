#pragma once

#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/thread/mutex.hpp>
#include <Utils/Point2of.h>

class PTracker;

class PTrackerROS
{
	private:
		static const int NUMBER_OF_LASER_SCANS = 1081;
		
		std::map<std::string,PTracking::Point2of> robotPoses;
		std::vector<ros::Subscriber> subscriberRobotPoses;
		std::vector<double> scanRangeData;
		ros::NodeHandle nodeHandle;
		ros::Subscriber subscriberLaserScan;
		PTracker* pTracker;
		boost::mutex mutex, mutexScan;
		double maxScanAngle, minScanAngle, maxReading, scanAngleIncrement;
		int agentId;
		
		bool checkObjectDetection(const PTracking::Point2of& robotPose, const PTracking::Point2of& objectPoseGlobalFrame);
		
	public:
		PTrackerROS();
		
		virtual ~PTrackerROS();
		
		void exec();
		void updateLaserScan(const sensor_msgs::LaserScan::ConstPtr& message);
		void updateRobotPose(const nav_msgs::Odometry::ConstPtr& message);
};
