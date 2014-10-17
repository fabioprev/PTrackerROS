#include "PTrackerROS.h"
#include <PTracker/PTracker.h>
#include <Utils/Utils.h>
#include <tf/LinearMath/Matrix3x3.h>

using namespace std;
using namespace PTracking;
using geometry_msgs::Quaternion;
using nav_msgs::Odometry;
using sensor_msgs::LaserScan;

PTrackerROS::PTrackerROS() : nodeHandle("~"), pTracker(0)
{
	int numberOfRobots;
	
	nodeHandle.getParam("agentId",agentId);
	nodeHandle.getParam("numberOfRobots",numberOfRobots);
	nodeHandle.getParam("maxReading",maxReading);
	
	for (int i = 0; i < numberOfRobots; ++i)
	{
		stringstream s;
		
		s << "/robot_" << i << "/base_pose_ground_truth";
		
		subscriberRobotPoses.push_back(nodeHandle.subscribe(s.str(),1024,&PTrackerROS::updateRobotPose,this));
	}
	
	subscriberLaserScan = nodeHandle.subscribe("scan",1024,&PTrackerROS::updateLaserScan,this);
	
	pTracker = new PTracker(agentId,string(getenv("PTracking_ROOT")) + string("/../config/Stage/parameters.cfg"));
}

PTrackerROS::~PTrackerROS()
{
	if (pTracker != 0) delete pTracker;
}

bool PTrackerROS::checkObjectDetection(const Point2of& robotPose, const Point2of& objectPoseGlobalFrame)
{
	static Point2of objectPoseRobotFrame;
	static float cosTheta, dx, dy, robotDiameter = 0.3, sinTheta;
	static int range = 5;
	static int minIntervalRange, maxIntervalRange;
	
	dx = objectPoseGlobalFrame.x - robotPose.x;
	dy = objectPoseGlobalFrame.y - robotPose.y;
	
	cosTheta = cos(robotPose.theta);
	sinTheta = sin(robotPose.theta);
	
	// Target position w.r.t. the mobile frame (robot frame).
	objectPoseRobotFrame.x =  cosTheta * dx + sinTheta * dy;
	objectPoseRobotFrame.y = -sinTheta * dx + cosTheta * dy;
	objectPoseRobotFrame.theta = atan2(objectPoseRobotFrame.y,objectPoseRobotFrame.x);
	
	mutexScan.lock();
	
	int index = (int) ((((objectPoseRobotFrame.theta - minScanAngle) / (maxScanAngle - minScanAngle)) * (maxScanAngle - minScanAngle)) / scanAngleIncrement);
	
	if ((index < 0) || (index >= NUMBER_OF_LASER_SCANS))
	{
		mutexScan.unlock();
		
		return false;
	}
	
	minIntervalRange = index - range;
	
	if (minIntervalRange < 0) minIntervalRange = 0;
	
	maxIntervalRange = index + range;
	
	if (maxIntervalRange >= NUMBER_OF_LASER_SCANS) maxIntervalRange = NUMBER_OF_LASER_SCANS - 1;
	
	Point2f objectPose;
	
	objectPose.x = objectPoseGlobalFrame.x;
	objectPose.y = objectPoseGlobalFrame.y;
	
	double theta;
	
	for (int i = minIntervalRange; i <= maxIntervalRange; ++i)
	{
		theta = minScanAngle + (scanAngleIncrement * i);
		
		const Point2of& scanPose = Utils::convertRelative2Global(Point2of(scanRangeData.at(i) * cos(theta),scanRangeData.at(i) * sin(theta),0),robotPose);
		
		if ((fabs(scanPose.x - objectPose.x) <= robotDiameter) && (fabs(scanPose.y - objectPose.y) <= robotDiameter))
		{
			mutexScan.unlock();
			
			return true;
		}
	}
	
	mutexScan.unlock();
	
	return false;
}

void PTrackerROS::exec()
{
	vector<ObjectSensorReading::Observation> obs;
	ObjectSensorReading visualReading;
	Point2of objectPoseRobotFrame;
	stringstream s;
	
	/// Stage starts the robot's enumeration from 0 while PTracking from 1.
	s << "/robot_" << (agentId - 1) << "/odom";
	
	mutex.lock();
	
	const map<string,Point2of>::const_iterator& robotPose = robotPoses.find(s.str());
	
	/// Something nasty has just happened.
	if (robotPose == robotPoses.end())
	{
		mutex.unlock();
		
		return;
	}
	
	for (map<string,Point2of>::const_iterator it = robotPoses.begin(); it != robotPoses.end(); ++it)
	{
		if (it->first == s.str()) continue;
		
		if (checkObjectDetection(robotPose->second,it->second))
		{
			WARN("Object detected!!!" << endl);
			
			ObjectSensorReading::Observation observation;
			
			observation.observation.rho = sqrt((it->second.x * it->second.x) + (it->second.y * it->second.y));
			observation.observation.theta = atan2(it->second.y,it->second.x);
			observation.head.x = it->second.x;
			observation.head.y = it->second.y;
			observation.model.barycenter = it->second.x;
			
			obs.push_back(observation);
		}
		else DEBUG("Object NOT detected" << endl);
	}
	
	mutex.unlock();
	
	visualReading.setObservations(obs);
	visualReading.setObservationsAgentPose(robotPose->second);
	
	pTracker->exec(visualReading);
}

void PTrackerROS::updateLaserScan(const LaserScan::ConstPtr& message)
{
	mutexScan.lock();
	
	scanRangeData.clear();
	
	maxScanAngle = message->angle_max;
	minScanAngle = message->angle_min;
	scanAngleIncrement = message->angle_increment;
	copy(&message->ranges[0],&message->ranges[NUMBER_OF_LASER_SCANS],back_inserter(scanRangeData));
	
	mutexScan.unlock();
}

void PTrackerROS::updateRobotPose(const Odometry::ConstPtr& message)
{
	Point2of currentRobotPose;
	double roll, pitch, yaw;
	
	const Quaternion& q = message->pose.pose.orientation;
	
	tf::Matrix3x3(tf::Quaternion(q.x,q.y,q.z,q.w)).getRPY(roll,pitch,yaw);
	
	mutex.lock();
	
	currentRobotPose.x = message->pose.pose.position.x;
	currentRobotPose.y = message->pose.pose.position.y;
	currentRobotPose.theta = yaw;
	
	const map<string,Point2of>::iterator& robotPose = robotPoses.find(message->header.frame_id);
	
	if (robotPose == robotPoses.end()) robotPoses.insert(make_pair(message->header.frame_id,currentRobotPose));
	else robotPose->second = currentRobotPose;
	
	mutex.unlock();
}
