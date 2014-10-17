#include "PTrackerROS.h"
#include <PTracker/PTracker.h>
#include <Utils/Utils.h>
#include <tf/LinearMath/Matrix3x3.h>

using namespace std;
using namespace PTracking;
using geometry_msgs::Quaternion;
using nav_msgs::Odometry;
using LaserScanDetector::ObjectDetection;
using LaserScanDetector::Object;

PTrackerROS::PTrackerROS() : nodeHandle("~"), pTracker(0)
{
	nodeHandle.getParam("agentId",agentId);
	
	subscriberObjectDetected = nodeHandle.subscribe("objectDetected",1024,&PTrackerROS::updateObjectDetected,this);
	subscriberRobotPose = nodeHandle.subscribe("base_pose_ground_truth",1024,&PTrackerROS::updateRobotPose,this);
	
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
	Point2of currentRobotPose;
	
	mutex.lock();
	
	currentRobotPose = robotPose;
	
	mutex.unlock();
	
	mutexDetection.lock();
	
	for (vector<Object>::const_iterator it = objectDetected.begin(); it != objectDetected.end(); ++it)
	{
		ObjectSensorReading::Observation observation;
		
		const Point2of& objectPose = Utils::convertRelative2Global(Point2of(it->x,it->y,0),currentRobotPose);
		
		observation.observation.rho = sqrt((objectPose.x * objectPose.x) + (objectPose.y * objectPose.y));
		observation.observation.theta = atan2(objectPose.y,objectPose.x);
		observation.head.x = objectPose.x;
		observation.head.y = objectPose.y;
		observation.model.barycenter = objectPose.x;
	
		obs.push_back(observation);
	}
	
	mutexDetection.unlock();
	
	visualReading.setObservations(obs);
	visualReading.setObservationsAgentPose(currentRobotPose);
	
	pTracker->exec(visualReading);
	
	objectDetected.clear();
}

void PTrackerROS::updateObjectDetected(const ObjectDetection::ConstPtr& message)
{
	mutexDetection.lock();
	
	objectDetected = message->objects;
	
	mutexDetection.unlock();
}

void PTrackerROS::updateRobotPose(const Odometry::ConstPtr& message)
{
	double roll, pitch, yaw;
	
	const Quaternion& q = message->pose.pose.orientation;
	
	tf::Matrix3x3(tf::Quaternion(q.x,q.y,q.z,q.w)).getRPY(roll,pitch,yaw);
	
	mutex.lock();
	
	robotPose.x = message->pose.pose.position.x;
	robotPose.y = message->pose.pose.position.y;
	robotPose.theta = yaw;
	
	mutex.unlock();
}
