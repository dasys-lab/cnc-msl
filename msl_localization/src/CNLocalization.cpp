#include <iostream>
#include <ros/ros.h>

#include "../include/msl_localization.h"

#include "RosMsgReceiver.h"
#include "MapHelper.h"

using namespace std;

namespace msl {

msl_actuator_msgs::RawOdometryInfoPtr OdometryInfoMsg;


double gaussian(double x) {
	static double sigq = 9;
	static double mean = 0;
	return exp(-((x-mean)*(x-mean))/(2*sigq))/(sqrt(2*3.141592*sigq));
}

}
int main(int argc, char** argv) {
	int scanSkip=1;
	ros::init(argc, argv, "CNLocalization");
	ros::NodeHandle n;
	RosMsgReceiver* rmr = RosMsgReceiver::getInstance();
	rmr->initialize();
	
	MapHelper* mh = MapHelper::getInstance();
	if(ros::ok()) mh->initializeMap();
	//Dont exchange the call order	
	msl_localization *loc = new msl_localization(800); //1200


	
	int iteration=0;
	while(ros::ok()) {
		ros::spinOnce();
		while(!rmr->dirty && ros::ok()) {
			ros::Duration(0.0001).sleep();
			ros::spinOnce();
		}
		if(!ros::ok()) break;
		
		if (rmr->reloc) {
			loc->resetStartParticle();
			rmr->reloc = false;
		}

		auto lpl = rmr->getCurrentLinePointList();

		//if(++iteration%20==0) loc->sendParticleCloud();
		loc->iterate(lpl, mh->getMap());
		rmr->dirty=false;
	}
	return 0;
}

