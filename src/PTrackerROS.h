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
		std::map<int,std::pair<int,std::pair<int,int> > > colorMap;
		std::vector<LaserScanDetector::Object> objectDetected;
		ros::NodeHandle nodeHandle;
		ros::Subscriber subscriberObjectDetected, subscriberRobotPose;
		boost::mutex mutex, mutexDetection;
		PTracking::Point2of robotPose;
		PTracker* pTracker;
		std::string distortion, setupFile;
		float gaussianNoiseX, gaussianNoiseY, gaussianNoiseTheta, falsePositiveBurstTime, packetLossProbability, startingDistortionDistance, trueNegativeBurstTime, worldSizeX, worldSizeY;
		int agentId, falsePositiveObservations, networkCoverage, trueNegativeObservations;
		bool isGaussianNoise;
		
		void addArtificialNoise(std::vector<PTracking::ObjectSensorReading::Observation>& obs);
		
		inline float gaussianNoise(float mean, float sigma) const
		{
			static boost::mt19937 rng;
			static boost::normal_distribution<> nd(mean,sigma);
			static boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > var_nor(rng,nd);
			
			return var_nor();
		}
		
	public:
		PTrackerROS();
		
		virtual ~PTrackerROS();
		
		void configure(const std::string& filename);
		void exec();
		void updateObjectDetected(const LaserScanDetector::ObjectDetection::ConstPtr& message);
		
#ifndef LOCALIZER
		void updateRobotPose(const nav_msgs::Odometry::ConstPtr& message);
#else
		void updateRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message);
#endif
};
