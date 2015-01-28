/*
 * MSLWorldModel.cpp
 *
 *  Created on: 27.10.2014
 *      Author: endy
 */

#include <GeometryCalculator.h>
#include "MSLWorldModel.h"
#include "HaveBall.h"
namespace msl {

MSLWorldModel* MSLWorldModel::get() {
	static MSLWorldModel instance;
	return &instance;
}

void MSLWorldModel::onSimulatorData(
		msl_simulator::messages_robocup_ssl_wrapperPtr msg) {
	if (simData.size() > ringBufferLength) {
		simData.pop_back();
	}
	simData.push_front(msg);
}

MSLWorldModel::MSLWorldModel() : haveBall(this) {
	ringBufferLength = 10;
	ownID = supplementary::SystemConfig::getOwnRobotID();
	spinner = new ros::AsyncSpinner(4);
	sub = n.subscribe("/MSLSimulator/MessagesRoboCupSSLWrapper", 10,
			&MSLWorldModel::onSimulatorData, (MSLWorldModel*) this);
	spinner->start();

	rawOdomSub = n.subscribe("/RawOdometry", 10,
			&MSLWorldModel::onRawOdometryInfo, (MSLWorldModel*) this);

	joystickSub = n.subscribe("/Joystick", 10,
			&MSLWorldModel::onJoystickCommand, (MSLWorldModel*) this);

	refereeBoxInfoBodySub = n.subscribe("/RefereeBoxInfoBody", 10,
			&MSLWorldModel::onRefereeBoxInfoBody, (MSLWorldModel*) this);
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
	} else {

		double x, y, oriantation;
		if (simData.size() > 0) {
			auto data = *simData.begin();
			for (auto& r : data->detection.robots_yellow) {
				if (r.robot_id == ownID) {
					x = r.x;
					y = r.y;
					oriantation = r.orientation;
				}
			}
			p = make_shared<CNPosition>();
			p->x = x;
			p->y = y;
			p->theta = oriantation;

		}

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
	} else {
		if (simData.size() > 0) {
			auto data = *simData.begin();
			if (data->detection.balls.size() > 0) {
				p = make_shared<CNPoint2D>(data->detection.balls.begin()->x,
						data->detection.balls.begin()->y);
				auto ownPos = this->getOwnPosition();
				if (ownPos.operator bool()) {
					p = p->alloToEgo(*ownPos);
				}
			}
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
}

msl_msgs::RefereeBoxInfoBodyPtr MSLWorldModel::getRefereeBoxInfoBody() {
	lock_guard<mutex> lock(refereeMutex);
		if (refereeBoxInfoBodyCommandData.size() == 0) {
			return nullptr;
		}
		return refereeBoxInfoBodyCommandData.front();
}

void MSLWorldModel::transformToWorldCoordinates(
		msl_sensor_msgs::WorldModelDataPtr& msg) {
}

bool MSLWorldModel::checkSituation(Situation situation) {
	auto ref = getRefereeBoxInfoBody();
	if(!ref) {
		return false;
	}
	if(ref->lastCommand == situation)
	{
		return true;
	}
	return false;
}
} /* namespace msl */

