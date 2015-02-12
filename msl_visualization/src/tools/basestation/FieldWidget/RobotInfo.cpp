/*
 * RobotInfo.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Stefan Jakob
 */

#include <RobotInfo.h>

RobotInfo::RobotInfo()
{
	// TODO Auto-generated constructor stub

}

RobotInfo::~RobotInfo()
{
	// TODO Auto-generated destructor stub
}

int RobotInfo::getId() const
{
	return id;
}

void RobotInfo::setId(int id)
{
	this->id = id;
}

boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> RobotInfo::getMsg()
{
	return msg;
}

void RobotInfo::setMsg(boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> msg)
{
	this->msg = msg;
}

unsigned long RobotInfo::getTimeStamp()
{
	return timeStamp;
}

void RobotInfo::setTimeStamp(unsigned long timeStamp)
{
	this->timeStamp = timeStamp;
}
