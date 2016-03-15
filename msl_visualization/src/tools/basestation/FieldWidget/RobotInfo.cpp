/*
 * RobotInfo.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Stefan Jakob
 */

#include <RobotInfo.h>
#include "FieldWidget3D.h"

RobotInfo::RobotInfo(FieldWidget3D* field)
{
	id = 0;
	timeStamp = 0;
	this->visualization = std::make_shared<RobotVisualization>(this, field);
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


std::shared_ptr<RobotVisualization> RobotInfo::getVisualization()
{
        return this->visualization;
}

const boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> RobotInfo::getSharedWorldInfo() const
{
	return msg;
}

void RobotInfo::setSharedWorldInfo(const boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> msg)
{
	this->msg = msg;
}

const boost::shared_ptr<msl_msgs::CorridorCheck>& RobotInfo::getCorridorCheckInfo() const
{
        return corridorCheckInfo;
}

void RobotInfo::setCorridorCheckInfo(const boost::shared_ptr<msl_msgs::CorridorCheck>& corridorCheckInfo = nullptr)
{
        this->corridorCheckInfo = corridorCheckInfo;
}

const boost::shared_ptr<msl_msgs::PathPlanner>& RobotInfo::getPathPlannerInfo() const
{
        return pathPlannerInfo;
}

void RobotInfo::setPathPlannerInfo(const boost::shared_ptr<msl_msgs::PathPlanner>& pathPlannerInfo = nullptr)
{
        this->pathPlannerInfo = pathPlannerInfo;
}

const boost::shared_ptr<msl_msgs::VoronoiNetInfo>& RobotInfo::getVoronoiNetInfo() const
{
        return voronoiNetInfo;
}

void RobotInfo::setVoronoiNetInfo(const boost::shared_ptr<msl_msgs::VoronoiNetInfo>& voronoiNetInfo = nullptr)
{
  this->voronoiNetInfo = voronoiNetInfo;
}

const boost::shared_ptr<msl_helper_msgs::DebugMsg>& RobotInfo::getDebugMsg() const
{
        return debugMsg;
}

void RobotInfo::setDebugMsg(const boost::shared_ptr<msl_helper_msgs::DebugMsg>& debugMsg = nullptr)
{
        this->debugMsg = debugMsg;

        auto tmp = ros::Time::now();
        this->debugMsgTimeStamp = (unsigned long)tmp.sec * (unsigned long)1000000000 + (unsigned long)tmp.nsec;
}

const boost::shared_ptr<msl_helper_msgs::PassMsg>& RobotInfo::getPassMsg() const
{
        return passMsg;
}

void RobotInfo::setPassMsg(const boost::shared_ptr<msl_helper_msgs::PassMsg>& passMsg)
{
          this->passMsg = passMsg;

          auto tmp = ros::Time::now();
          this->passMsgTimeStamp = (unsigned long)tmp.sec * (unsigned long)1000000000 + (unsigned long)tmp.nsec;
}

unsigned long RobotInfo::getTimeStamp()
{
	return timeStamp;
}

void RobotInfo::updateTimeStamp()
{
        auto tmp = ros::Time::now();
        this->timeStamp = (unsigned long)tmp.sec * (unsigned long)1000000000 + (unsigned long)tmp.nsec;
}

bool RobotInfo::isTimeout()
{
        auto tmp = ros::Time::now();
        unsigned long now = (unsigned long)tmp.sec * (unsigned long)1000000000 + (unsigned long)tmp.nsec;

        return (now - this->timeStamp) > 2000000000;
}

bool RobotInfo::isDebugMsgTimeout()
{
        if (false == this->debugMsg)
                return true;

        auto tmp = ros::Time::now();
        unsigned long now = (unsigned long)tmp.sec * (unsigned long)1000000000 + (unsigned long)tmp.nsec;

        return (now - this->debugMsgTimeStamp) > 2000000000;
}

bool RobotInfo::isPassMsgTimeout()
{
        if (false == this->passMsg)
                return true;

        auto tmp = ros::Time::now();
        unsigned long now = (unsigned long)tmp.sec * (unsigned long)1000000000 + (unsigned long)tmp.nsec;

        return (now - this->passMsgTimeStamp) > this->passMsg->validFor;
}

