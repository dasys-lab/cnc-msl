#include "RosMsgReceiver.h"
#include <iostream>
//#include <shared_ptr.h>


RosMsgReceiver * RosMsgReceiver::instance = NULL;

RosMsgReceiver * RosMsgReceiver::getInstance(){
	if(instance == NULL)
		instance = new RosMsgReceiver();
	return instance;
}


void RosMsgReceiver::initialize() {
	mpReceived = false;
	scnReceived = false;
	pseReceived = false;
	reloc = false;

	ros::NodeHandle node;

	Mapsub = node.subscribe<nav_msgs::OccupancyGrid, RosMsgReceiver>("/map", 1, &RosMsgReceiver::handleMapMessage, this);
	LaserSub = node.subscribe<sensor_msgs::LaserScan, RosMsgReceiver>("/scan", 1, &RosMsgReceiver::handleScanMessage, this);
	Iniposesub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped, RosMsgReceiver>("/initialpose", 1, &RosMsgReceiver::handlePoseMessage, this);
	OdometrySub = node.subscribe("/RawOdometry", 10, &RosMsgReceiver::handleOdometryInfoMessage, this);
    RelocSub = node.subscribe("CNActuator/VisionRelocTrigger",
				1, &RosMsgReceiver::handleVisionRelocTriggerMessage, this);
    LinePointListSub = node.subscribe("/CNVision/LinePointList", 1, &RosMsgReceiver::handleLinePointListMessage, this);
    imuDataSub = node.subscribe("/IMUData", 1, &RosMsgReceiver::handleIMUData, this);

	particlepub = node.advertise<geometry_msgs::PoseArray>("/particlecloud", 1);
	coipub = node.advertise<msl_sensor_msgs::CorrectedOdometryInfo>("/CorrectedOdometryInfo", 1);


	odometryInfoMsg = boost::make_shared<msl_actuator_msgs::RawOdometryInfo>();
	odometryInfoMsg->motion.angle = 0;
	odometryInfoMsg->motion.rotation = 0;
	odometryInfoMsg->motion.translation = 0;
	odometryInfoMsg->position.angle = 0;
	odometryInfoMsg->position.x = 0;
	odometryInfoMsg->position.y = 0;
	odometryInfoMsg->position.certainty = 0;
	//spinner = new ros::AsyncSpinner(1);
	//spinner->start();
}


nav_msgs::MapMetaData* RosMsgReceiver::getMapInfo() {
	return &mapInfo;
}


unsigned char* RosMsgReceiver::getMap() {
	return map;
}


void RosMsgReceiver::handleScanMessage(const sensor_msgs::LaserScan::ConstPtr& scan) {
	 std::cout << "Laserscan Received" << std::endl;
	 msgptr = scan;
	 dirty=true;
	 scnReceived = true;
	 observTime = msgptr->header.stamp;
}


void RosMsgReceiver::handlePoseMessage(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {
	std::cout << "PoseMessage" << std::endl;
	poseptr = pose;
	pseReceived = true;
	dirty=true;
}


void RosMsgReceiver::handleMapMessage(const nav_msgs::OccupancyGrid::ConstPtr& message) {
	if(!mpReceived) {
		mapInfo = message->info;
		map = new unsigned char[message->info.width*message->info.height];
		memcpy(map, &message->data[0], message->info.width*message->info.height*sizeof(unsigned char));
		mpReceived = true;
	}
}

void RosMsgReceiver::handleIMUData(msl_actuator_msgs::IMUDataPtr msg)
{
	this->imuData = msg;
}

void RosMsgReceiver::handleVisionRelocTriggerMessage(const
		msl_actuator_msgs::VisionRelocTrigger::ConstPtr& msg) {
	if(supplementary::SystemConfig::getOwnRobotID() != msg->receiverID) return;
	reloc = true;
}

void RosMsgReceiver::sendParticleCloud(geometry_msgs::PoseArray &p) {
	particlepub.publish(p);
}


void RosMsgReceiver::handleOdometryInfoMessage(msl_actuator_msgs::RawOdometryInfoPtr msg)
{
	odometryInfoMsg = msg;
}

void RosMsgReceiver::handleLinePointListMessage(msl_sensor_msgs::LinePointListPtr msg) {
	currentLinePoints = msg;
	cout << "Received LinepointList" << endl;
	dirty=true;
}

msl_sensor_msgs::LinePointListPtr RosMsgReceiver::getCurrentLinePointList() {
	return currentLinePoints;
}

msl_actuator_msgs::RawOdometryInfoPtr RosMsgReceiver::getOdometryInfo()
{
	return odometryInfoMsg;
}

msl_actuator_msgs::IMUDataPtr RosMsgReceiver::getIMUData() {
	return imuData;
}
