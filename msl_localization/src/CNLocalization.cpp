#include <iostream>
#include <ros/ros.h>


#include "RosMsgReceiver.h"
#include "MapHelper.h"
#include "ParticleFilter.h"

using namespace std;



double gaussian(double x) {
	static double sigq = 9;
	static double mean = 0;
	return exp(-((x-mean)*(x-mean))/(2*sigq))/(sqrt(2*3.141592*sigq));
}

int main(int argc, char** argv) {
	int scanSkip=1;
	ros::init(argc, argv, "CNLocalization");
	RosMsgReceiver* rmr = RosMsgReceiver::getInstance();
	rmr->initialize();
	
	MapHelper* mh = MapHelper::getInstance();
	
	ParticleFilter *pf = new ParticleFilter(2); //1200
	tf::TransformListener listener;
	
	while(ros::ok()) {
		ros::spinOnce();
		if(rmr->mapReceived()) break;
	}
	if(ros::ok()) mh->initializeMap();
	
	while(ros::ok()) {
		ros::spinOnce();
		if(rmr->scanReceived()) break;
	}
	std::vector<LinePoint> ll;
	if(rmr->scanReceived()) ll.resize(rmr->getScan()->ranges.size()/scanSkip);
	LinePoint lp;
	
	int iteration=0;
	tf::StampedTransform transform;
	while(ros::ok()) {
		ros::spinOnce();
		while(!rmr->dirty && ros::ok()) {
			ros::Duration(0.0001).sleep();
			ros::spinOnce();
		}
		if(!ros::ok()) break;
		
		try{
			listener.lookupTransform("/base_link", "/laser",  ros::Time(0), transform);
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
		}
		
		for(int i=0; i<rmr->getScan()->ranges.size()/scanSkip; i++) {
			double angle = (rmr->getScan()->angle_min + scanSkip*i*rmr->getScan()->angle_increment) + transform.getRotation().getAngle();
			lp.x = (cos(angle) * rmr->getScan()->ranges[scanSkip*i]) + transform.getOrigin().getX();
			lp.y = (sin(angle) * rmr->getScan()->ranges[scanSkip*i]) + transform.getOrigin().getY();
			ll[i] =  lp;
		}
		
		
		if(rmr->poseReceived()) {
			pf->initParticles(rmr->getPose()->pose.pose.position.x, rmr->getPose()->pose.pose.position.y, 
				2*atan2(rmr->getPose()->pose.pose.orientation.z, rmr->getPose()->pose.pose.orientation.w),
				0.7, 0.7, 0.3);
			rmr->poseProcessed();
			pf->sendParticleCloud();
		}
		
		if(++iteration%20==0) pf->sendParticleCloud();
		pf->iterate(ll, mh->getMap());
		rmr->dirty=false;
	}
	return 0;
}