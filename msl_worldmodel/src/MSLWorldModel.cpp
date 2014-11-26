/*
 * MSLWorldModel.cpp
 *
 *  Created on: 27.10.2014
 *      Author: endy
 */

#include "MSLWorldModel.h"
#include "GeometryTransformer.h"

namespace msl {

	MSLWorldModel* MSLWorldModel::get() {
		static MSLWorldModel instance;
		return &instance;
	}

	void MSLWorldModel::onSimulatorData(
			msl_simulator::messages_robocup_ssl_wrapperPtr msg) {
		if (simData.size() > 10) {
			simData.pop_back();
		}
		simData.push_front(msg);
	}

	MSLWorldModel::MSLWorldModel() {
		hasBallIteration = 0;
		ownID = supplementary::SystemConfig::getOwnRobotID();
		spinner = new ros::AsyncSpinner(4);
		sub = n.subscribe("/MSLSimulator/MessagesRoboCupSSLWrapper", 10,
				&MSLWorldModel::onSimulatorData, (MSLWorldModel*) this);
		spinner->start();
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
	CNPosition MSLWorldModel::getOwnPosition() {

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
		}
		CNPosition p;
		p.x=x;
		p.y=y;
		p.theta = oriantation;
		return p;
	}

	CNPoint2D MSLWorldModel::getBallPosition() {
		CNPoint2D ret;
		if (simData.size() > 0) {
			auto data = *simData.begin();
			if (data->detection.balls.size() > 0) {
				ret.x = data->detection.balls.begin()->x;
				ret.y = data->detection.balls.begin()->y;
			}
		}
		return ret;
	}

	bool MSLWorldModel::haveBall() {
		CNPoint2D alloBallPos = this->getBallPosition();
		CNPosition ownPos = this->getOwnPosition();
		CNPoint2D egoBallPos = alloBallPos.alloToEgo(ownPos);

		if (fabs(egoBallPos.x) <= 125 && fabs(egoBallPos.y) <= 125
					&& fabs(atan2(egoBallPos.y, egoBallPos.x)) <= 0.075) {
			hasBallIteration++;
		}
		else
		{
			hasBallIteration = 0;
		}

		if(hasBallIteration >= 5) {
			return true;
		}else {
			return false;
		}
	}

	bool MSLWorldModel::nearPoint(CNPoint2D alloTargetPos) {
		CNPosition ownPos = this->getOwnPosition();
		CNPoint2D egoTargetPos = alloTargetPos.alloToEgo(ownPos);


		if (fabs(egoTargetPos.x) <= 300 && fabs(egoTargetPos.y) <= 300) {
			return true;
		}
		return false;
	}

} /* namespace msl */

