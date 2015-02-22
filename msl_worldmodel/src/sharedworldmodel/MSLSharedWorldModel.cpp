/*
 * MSLSharedWorldModel.cpp
 *
 *  Created on: 11.02.2015
 *      Author: tobi
 */

#include <ios>
#include "sharedworldmodel/MSLSharedWorldModel.h"
#include "msl_sensor_msgs/SharedWorldInfo.h"
#include "MSLWorldModel.h"

using namespace std;

namespace msl {

MSLSharedWorldModel::MSLSharedWorldModel(MSLWorldModel* wm) {

	this->wm = wm;
	timer = n.createTimer(ros::Duration(0.1), &MSLSharedWorldModel::sendSharedWorldModelData, this);

	sc = supplementary::SystemConfig::getInstance();
	ownID = sc->getOwnRobotID();

	sharedWolrdModelPub = n.advertise<msl_sensor_msgs::SharedWorldInfo>(
					"/SharedWorldInfo", 2);



}

MSLSharedWorldModel::~MSLSharedWorldModel() {
	// TODO Auto-generated destructor stub
}

void MSLSharedWorldModel::sendSharedWorldModelData(const ros::TimerEvent& e) {
	wm->sendSharedWorldModelData();
}

} /* namespace msl */
