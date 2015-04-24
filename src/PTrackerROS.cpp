#include "PTrackerROS.h"
#include <Manfield/ConfigFile/ConfigFile.h>
#include <PTracker/PTracker.h>
#include <Utils/Utils.h>
#include <tf/LinearMath/Matrix3x3.h>

using namespace std;
using namespace PTracking;
using GMapping::ConfigFile;
using geometry_msgs::Quaternion;
using geometry_msgs::PoseWithCovarianceStamped;
using nav_msgs::Odometry;
using LaserScanDetector::ObjectDetection;
using LaserScanDetector::Object;

PTrackerROS::PTrackerROS() : nodeHandle("~"), pTracker(0)
{
	nodeHandle.getParam("agentId",agentId);
	nodeHandle.getParam("setupFile",setupFile);
	
	subscriberObjectDetected = nodeHandle.subscribe("objectDetected",1024,&PTrackerROS::updateObjectDetected,this);
	subscriberRobotPose = nodeHandle.subscribe("base_pose_ground_truth",1024,&PTrackerROS::updateRobotPose,this);
	
	pTracker = new PTracker(agentId,string(getenv("PTracking_ROOT")) + string("/../config/Stage/parameters.cfg"));
	
	int counter = 1;
	
	for (int i = 0; i < 256; i += 63)
	{
		for (int j = 0; j < 256; j += 63)
		{
			for (int k = 0; k < 256; k += 128, ++counter)
			{
				colorMap.insert(make_pair(counter,make_pair(i,make_pair(j,k))));
			}
		}
	}
	
	configure(setupFile);
}

PTrackerROS::~PTrackerROS()
{
	if (pTracker != 0) delete pTracker;
}

void PTrackerROS::addArtificialNoise(vector<ObjectSensorReading::Observation>& obs)
{
	/// Adding Gaussian noise to observations.
	if (isGaussianNoise)
	{
		for (vector<ObjectSensorReading::Observation>::iterator it = obs.begin(); it != obs.end(); ++it)
		{
			it->observation.x += gaussianNoise(gaussianNoiseX,0.3);
			it->observation.y += gaussianNoise(gaussianNoiseY,0.3);
		}
	}
	
	/// Adding false positive observations for a burst period of time equals to falsePositiveBurstTime.
	if ((falsePositiveBurstTime > 0.0) && (falsePositiveObservations > 0))
	{
		static const int TIME_TO_WAIT_BEFORE_NEXT_BURST = 10000 - falsePositiveBurstTime;
		
		static vector<ObjectSensorReading::Observation> falseObservations;
		static Timestamp initialTimestamp, initialTimestampBurst;
		static bool isBurstTime = false, isGenerationTime = true;
		
		if (isBurstTime)
		{
			if ((Timestamp() - initialTimestampBurst).getMs() < falsePositiveBurstTime)
			{
				if (isGenerationTime)
				{
					falseObservations.clear();
					
					isGenerationTime = false;
					
					for (int i = 1; i <= falsePositiveObservations; ++i)
					{
						ObjectSensorReading::Observation falseObservation;
						
						falseObservation.observation.x = Utils::randr(worldSizeX);
						falseObservation.observation.y = Utils::randr(worldSizeY);
						falseObservation.head.x = falseObservation.observation.x;
						falseObservation.head.y = falseObservation.observation.y;
						falseObservation.model.barycenter = falseObservation.observation.x;
						
						falseObservations.push_back(falseObservation);
					}
				}
				
				for (vector<ObjectSensorReading::Observation>::const_iterator it = falseObservations.begin(); it != falseObservations.end(); ++it)
				{
					ObjectSensorReading::Observation falseObservation;
					
					falseObservation.observation.x = it->observation.x + gaussianNoise(gaussianNoiseX,0.3);
					falseObservation.observation.y = it->observation.y + gaussianNoise(gaussianNoiseY,0.3);
					
					obs.push_back(falseObservation);
				}
			}
			else
			{
				initialTimestamp.setToNow();
				isBurstTime = false;
			}
		}
		else
		{
			if ((TIME_TO_WAIT_BEFORE_NEXT_BURST - (Timestamp() - initialTimestamp).getMs()) < 0.0)
			{
				initialTimestampBurst.setToNow();
				isBurstTime = true;
				isGenerationTime = true;
			}
		}
	}
	
	if (strcasecmp(distortion.c_str(),"none") != 0)
	{
		Point2of currentRobotPose;
		
		mutex.lock();
		
		currentRobotPose = robotPose;
		
		mutex.unlock();
		
		/// Adding sensor's distortion. Set this parameter to 'none' for having an ideal sensor.
		for (vector<ObjectSensorReading::Observation>::iterator it = obs.begin(); it != obs.end(); ++it)
		{
			if (strcasecmp(distortion.c_str(),"linear") == 0)
			{
				float displacementX, displacementY;
				
				displacementX = currentRobotPose.x - it->observation.x;
				displacementY = currentRobotPose.y - it->observation.y;
				
				if (sqrt(pow(displacementX,2) + pow(displacementY,2)) > startingDistortionDistance)
				{
					it->observation.x += (displacementX * 0.1);
					it->observation.y += (displacementY * 0.1);
				}
			}
		}
	}
}

void PTrackerROS::configure(const string& filename)
{
	ConfigFile fCfg;
	string key, section, temp;
	float worldXMin, worldXMax, worldYMin, worldYMax;
	
	if (!fCfg.read(filename))
	{
		ERR("Error reading file '" << filename << "' for PTrackerROS configuration. Exiting..." << endl);
		
		exit(-1);
	}
	
	try
	{
		section = "Network";
		
		key = "networkCoverage";
		temp = string(fCfg.value(section,key));
		
		if (temp == "inf") networkCoverage = INT_MAX;
		else networkCoverage = atoi(temp.c_str());
		
		key = "packetLossProbability";
		packetLossProbability = fCfg.value(section,key);
		
		section = "Sensor";
		
		key = "distortion";
		distortion = string(fCfg.value(section,key));
		
		key = "startingDistortionDistance";
		startingDistortionDistance = fCfg.value(section,key);
		
		section = "Observation";
		
		key = "gaussianNoise";
		isGaussianNoise = fCfg.value(section,key);
		
		key = "gaussianNoiseX";
		gaussianNoiseX = fCfg.value(section,key);
		
		key = "gaussianNoiseY";
		gaussianNoiseY = fCfg.value(section,key);
		
		key = "gaussianNoiseTheta";
		gaussianNoiseTheta = fCfg.value(section,key);
		
		key = "falsePositiveBurstTime";
		falsePositiveBurstTime = fCfg.value(section,key);
		
		key = "falsePositiveObservations";
		falsePositiveObservations = fCfg.value(section,key);
		
		key = "trueNegativeBurstTime";
		trueNegativeBurstTime = fCfg.value(section,key);
		
		key = "trueNegativeObservations";
		trueNegativeObservations = fCfg.value(section,key);
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	if (!fCfg.read(string(getenv("PTracking_ROOT")) + string("/../config/Stage/parameters.cfg")))
	{
		ERR("Error reading file '" << string(getenv("PTracking_ROOT")) + string("/../config/Stage/parameters.cfg") << "' for PTrackerROS configuration. Exiting..." << endl);
		
		exit(-1);
	}
	
	try
	{
		section = "location";
		
		key = "worldXMin";
		temp = string(fCfg.value(section,key));
		
		if (temp == "-inf") worldXMin = -FLT_MAX;
		else worldXMin = atof(temp.c_str());
		
		key = "worldXMax";
		temp = string(fCfg.value(section,key));
		
		if (temp == "inf") worldXMax = FLT_MAX;
		else worldXMax = atof(temp.c_str());
		
		key = "worldYMin";
		temp = string(fCfg.value(section,key));
		
		if (temp == "-inf") worldYMin = -FLT_MAX;
		else worldYMin = atof(temp.c_str());
		
		key = "worldYMax";
		temp = string(fCfg.value(section,key));
		
		if (temp == "inf") worldYMax = FLT_MAX;
		else worldYMax = atof(temp.c_str());
		
		worldSizeX = worldXMax - worldXMin;
		worldSizeY = worldYMax - worldYMin;
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	ERR(endl << "******************************************************" << endl);
	DEBUG("PTrackerROS parameters:" << endl);
	
	if (networkCoverage == INT_MAX) INFO("\tNetwork coverage: inf" << endl)
	else INFO("\tNetwork coverage: " << networkCoverage << " (in meters)" << endl)
	
	WARN("\tProbability to loss a packet: " << packetLossProbability << endl);
	LOG("\tSensor distortion: " << distortion << endl);
	LOG("\tStarting distortion distance: " << startingDistortionDistance << endl);
	INFO("\tGaussian noise: [" << gaussianNoiseX << "," << gaussianNoiseY << "," << gaussianNoiseTheta << "]" << endl);
	WARN("\tFalse positive burst time: " << falsePositiveBurstTime << " (in ms)" << endl);
	WARN("\tNumber of false positive observations: " << falsePositiveObservations << endl);
	DEBUG("\tTrue negative burst time: " << trueNegativeBurstTime << " (in ms)" << endl);
	DEBUG("\tNumber of true negative observations: " << trueNegativeObservations << endl);
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
		
		observation.observation.x = objectPose.x;
		observation.observation.y = objectPose.y;
		observation.head.x = objectPose.x;
		observation.head.y = objectPose.y;
		observation.model.barycenter = objectPose.x;
		
		const map<int,pair<int,pair<int,int> > >::const_iterator& colorTrack = colorMap.find(it->id);
		
		if (colorTrack != colorMap.end())
		{
			observation.model.histograms[0][colorTrack->second.first] = (float) colorTrack->second.first / (ObjectSensorReading::Model::HISTOGRAM_VECTOR_LENGTH - 1);
			observation.model.histograms[1][colorTrack->second.second.first] = (float) colorTrack->second.second.first / (ObjectSensorReading::Model::HISTOGRAM_VECTOR_LENGTH - 1);
			observation.model.histograms[2][colorTrack->second.second.second] = (float) colorTrack->second.second.second / (ObjectSensorReading::Model::HISTOGRAM_VECTOR_LENGTH - 1);
		}
		
		obs.push_back(observation);
	}
	
	mutexDetection.unlock();
	
#if 0
	INFO("Obs (before): " << endl);
	
	for (vector<ObjectSensorReading::Observation>::iterator it = obs.begin(); it != obs.end(); ++it)
	{
		WARN("[" << it->observation.x << "," << it->observation.y << "]" << endl);
	}
	
	addArtificialNoise(obs);
	
	LOG("Obs (before): " << endl);
	
	for (vector<ObjectSensorReading::Observation>::iterator it = obs.begin(); it != obs.end(); ++it)
	{
		DEBUG("[" << it->observation.x << "," << it->observation.y << "]" << endl);
	}
	
	ERR("*********************************************" << endl);
#endif
	
	visualReading.setObservations(obs);
	visualReading.setObservationsAgentPose(currentRobotPose);
	
	pTracker->setNetworkModel(networkCoverage,packetLossProbability);
	pTracker->exec(visualReading);
	
	objectDetected.clear();
}

void PTrackerROS::updateObjectDetected(const ObjectDetection::ConstPtr& message)
{
	mutexDetection.lock();
	
	objectDetected = message->objects;
	
	mutexDetection.unlock();
}

#ifndef LOCALIZER
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
#else
	void PTrackerROS::updateRobotPose(const PoseWithCovarianceStamped::ConstPtr& message)
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
#endif
