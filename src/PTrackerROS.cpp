#include "PTrackerROS.h"
#include <PTracker/PTracker.h>
#include <Utils/Utils.h>
#include <tf/LinearMath/Matrix3x3.h>

using namespace std;
using namespace PTracking;
using geometry_msgs::Quaternion;
using nav_msgs::Odometry;

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
	
	pTracker = new PTracker(agentId,string(getenv("PTracking_ROOT")) + string("/../config/Stage/parameters.cfg"));
}

PTrackerROS::~PTrackerROS()
{
	if (pTracker != 0) delete pTracker;
}

void PTrackerROS::exec()
{
	vector<ObjectSensorReading::Observation> obs;
	ObjectSensorReading visualReading;
	Point2of objectPoseRobotFrame;
	stringstream s;
	float cosTheta, dx, dy, sinTheta;
	
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
		
		dx = it->second.x - robotPose->second.x;
		dy = it->second.y - robotPose->second.y;
		
		cosTheta = cos(robotPose->second.theta);
		sinTheta = sin(robotPose->second.theta);
		
		// Target position w.r.t. the mobile frame (robot frame).
		objectPoseRobotFrame.x =  cosTheta * dx + sinTheta * dy;
		objectPoseRobotFrame.y = -sinTheta * dx + cosTheta * dy;
		objectPoseRobotFrame.theta = atan2(objectPoseRobotFrame.y,objectPoseRobotFrame.x);
		
		if ((objectPoseRobotFrame.x > 0) && (objectPoseRobotFrame.mod() < maxReading))
		{
			ObjectSensorReading::Observation observation;
			
			observation.observation.rho = sqrt((it->second.x * it->second.x) + (it->second.y * it->second.y));
			observation.observation.theta = atan2(it->second.y,it->second.x);
			observation.head.x = it->second.x;
			observation.head.y = it->second.y;
			observation.model.barycenter = it->second.x;
			
			obs.push_back(observation);
		}
	}
	
	mutex.unlock();
	
	visualReading.setObservations(obs);
	visualReading.setObservationsAgentPose(Point2of(0.0,0.0,0.0));
	
	pTracker->exec(visualReading);
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
