/*
 * MSLWorldModel.cpp
 *
 *  Created on: 27.10.2014
 *      Author: Andreas Witsch
 */

#include <GeometryCalculator.h>
#include "MSLWorldModel.h"
#include "HaveBall.h"
#include "sharedworldmodel/MSLSharedWorldModel.h"
#include "RawSensorData.h"
#include "msl_sensor_msgs/SharedWorldInfo.h"
namespace msl
{

	MSLWorldModel* MSLWorldModel::get()
	{
		static MSLWorldModel instance;
		return &instance;
	}

	MSLWorldModel::MSLWorldModel() :
			ringBufferLength(10), haveBall(this), rawSensorData(this, 10)
	{
		ownID = supplementary::SystemConfig::getOwnRobotID();
		spinner = new ros::AsyncSpinner(4);
		spinner->start();

	rawOdomSub = n.subscribe("/RawOdometry", 10,
			&MSLWorldModel::onRawOdometryInfo, (MSLWorldModel*) this);

	joystickSub = n.subscribe("/Joystick", 10,
			&MSLWorldModel::onJoystickCommand, (MSLWorldModel*) this);

	refereeBoxInfoBodySub = n.subscribe("/RefereeBoxInfoBody", 10,
			&MSLWorldModel::onRefereeBoxInfoBody, (MSLWorldModel*) this);

	wmDataSub = n.subscribe("/WorldModel/WorldModelData", 10,
			&MSLWorldModel::onWorldModelData, (MSLWorldModel*) this);

	this->sharedWolrdModel = new MSLSharedWorldModel(this);

}
void MSLWorldModel::onJoystickCommand(msl_msgs::JoystickCommandPtr msg) {
	if (msg->robotId == this->ownID) {
		lock_guard<mutex> lock(joystickMutex);
		if (joystickCommandData.size() > ringBufferLength) {
			joystickCommandData.pop_back();
		}
		joystickCommandData.push_front(msg);
	}
}
msl_msgs::JoystickCommandPtr MSLWorldModel::getJoystickCommandInfo() {
	lock_guard<mutex> lock(joystickMutex);
	if (joystickCommandData.size() == 0) {
		return nullptr;
	}
	return joystickCommandData.front();
}
void MSLWorldModel::onRawOdometryInfo(
		msl_actuator_msgs::RawOdometryInfoPtr msg) {
	lock_guard<mutex> lock(rawOdometryMutex);
	if (rawOdometryData.size() > ringBufferLength) {
		rawOdometryData.pop_back();
	}
	rawOdometryData.push_front(msg);
}

msl_actuator_msgs::RawOdometryInfoPtr MSLWorldModel::getRawOdometryInfo() {
	lock_guard<mutex> lock(rawOdometryMutex);
	if (rawOdometryData.size() == 0) {
		return nullptr;
	}
	return rawOdometryData.front();
}

void MSLWorldModel::onWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg) {
	lock_guard<mutex> lock(wmMutex);
	if (wmData.size() > ringBufferLength) {

		wmData.pop_back();
	}
	transformToWorldCoordinates(msg);
	wmData.push_front(msg);
}

msl_sensor_msgs::WorldModelDataPtr MSLWorldModel::getWorldModelData() {
	lock_guard<mutex> lock(wmMutex);
	if (wmData.size() == 0) {
		return nullptr;
	}
	return wmData.front();
}

MSLWorldModel::~MSLWorldModel() {
	spinner->stop();
	delete spinner;
	delete this->sharedWolrdModel;
}

/**
 * returns x,y orientation:
 * (0,0) Is the field center
 * positive y = right side of the field (playing direction from yellow to blue)
 * negative x = blue goal
 * orientation of -pi = towards blue side
 *
 */
shared_ptr<CNPosition> MSLWorldModel::getOwnPosition() {
	msl_sensor_msgs::WorldModelDataPtr wmd;
	{
		lock_guard<mutex> lock(wmMutex);
		if (wmData.size() > 0) {
			wmd = *wmData.begin();
		}
	}
	shared_ptr<CNPosition> p;
	if (wmd.operator bool()) {
		p = make_shared<CNPosition>();
		p->x = wmd->odometry.position.x;
		p->y = wmd->odometry.position.y;
		p->theta = wmd->odometry.position.angle;
		return p;
	}
	return p;
}

shared_ptr<CNPoint2D> MSLWorldModel::getAlloBallPosition() {
	shared_ptr<CNPoint2D> p;
	auto ownPos = this->getOwnPosition();
	if (ownPos.operator bool()) {
		p = this->getEgoBallPosition();
		p = p->egoToAllo(*ownPos);
	}
	return p;
}

shared_ptr<CNPoint2D> MSLWorldModel::getEgoBallPosition() {
	msl_sensor_msgs::WorldModelDataPtr wmd;
	{
		lock_guard<mutex> lock(wmMutex);
		if (wmData.size() > 0) {
			wmd = *wmData.begin();
		}
	}
	shared_ptr<CNPoint2D> p;
	if (wmd.operator bool()) {
		if (wmd->ball.confidence > 0) {
			p = make_shared<CNPoint2D>(wmd->ball.point.x, wmd->ball.point.y);
		}
	}
	return p;
}


double MSLWorldModel::getKickerVoltage() {
	return this->kickerVoltage;
}

void MSLWorldModel::setKickerVoltage(double voltage) {
	this->kickerVoltage = voltage;
}

void MSLWorldModel::onRefereeBoxInfoBody(msl_msgs::RefereeBoxInfoBodyPtr msg) {
	lock_guard<mutex> lock(refereeMutex);
			if (refereeBoxInfoBodyCommandData.size() > ringBufferLength) {
				refereeBoxInfoBodyCommandData.pop_back();

			}
			refereeBoxInfoBodyCommandData.push_front(msg);


			if((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::start) {
				currentSituation = Situation::Start;
			} else if((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::stop) {
				currentSituation = Situation::Stop;
			} else if((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::command_joystick) {
				currentSituation = Situation::Joystick;
			}
}

msl_msgs::RefereeBoxInfoBodyPtr MSLWorldModel::getRefereeBoxInfoBody() {
	lock_guard<mutex> lock(refereeMutex);
		if (refereeBoxInfoBodyCommandData.size() == 0) {
			return nullptr;
		}
		return refereeBoxInfoBodyCommandData.front();
}

MSLSharedWorldModel* MSLWorldModel::getSharedWolrdModel() {
	return this->sharedWolrdModel;
}

void MSLWorldModel::transformToWorldCoordinates(
		msl_sensor_msgs::WorldModelDataPtr& msg) {
}

bool MSLWorldModel::checkSituation(Situation situation) {
	lock_guard<mutex> lock(situationChecker);
	return currentSituation == situation;
}
} /* namespace msl */

