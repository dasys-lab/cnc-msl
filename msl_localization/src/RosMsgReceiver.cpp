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
	ros::NodeHandle node;
	
	Mapsub = node.subscribe<nav_msgs::OccupancyGrid, RosMsgReceiver>("/map", 1, &RosMsgReceiver::handleMapMessage, (this));
	LaserSub = node.subscribe<sensor_msgs::LaserScan, RosMsgReceiver>("/scan", 1, &RosMsgReceiver::handleScanMessage, (this));
	Iniposesub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped, RosMsgReceiver>("/initialpose", 1, &RosMsgReceiver::handlePoseMessage, (this));
	
	particlepub = node.advertise<geometry_msgs::PoseArray>("/particlecloud", 1);
	
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
		//dirty=true;
	}
}


void RosMsgReceiver::sendParticleCloud(geometry_msgs::PoseArray &p) {
	particlepub.publish(p);
}

