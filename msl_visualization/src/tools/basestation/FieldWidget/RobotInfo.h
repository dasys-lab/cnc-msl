/*
 * RobotInfo.h
 *
 *  Created on: Feb 12, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_VISUALIZATION_SRC_TOOLS_BASESTATION_FIELDWIDGET_ROBOTINFO_H_
#define CNC_MSL_MSL_VISUALIZATION_SRC_TOOLS_BASESTATION_FIELDWIDGET_ROBOTINFO_H_

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <map>
#include <memory>
#include <vector>

#include <msl_sensor_msgs/SharedWorldInfo.h>
#include <msl_helper_msgs/DebugMsg.h>
#include <msl_helper_msgs/PassMsg.h>
#include <msl_msgs/CorridorCheck.h>
#include <msl_msgs/PathPlanner.h>
#include <msl_msgs/VoronoiNetInfo.h>

#include "src/tools/basestation/RobotVisualization/RobotVisualization.h"

class FieldWidget3D;

struct DebugMsg
{
        DebugMsg()
        {

        }
        DebugMsg(boost::shared_ptr<msl_helper_msgs::DebugMsg> msg, unsigned long timeStamp) : msg(msg), timeStamp(timeStamp)
        {

        }
        unsigned long timeStamp;
        boost::shared_ptr<msl_helper_msgs::DebugMsg> msg;
};

class RobotInfo
{
public:
	RobotInfo(FieldWidget3D* field);
	virtual ~RobotInfo();

	int getId() const;
	void setId(int id);
	std::string getName() const;
	void setName(std::string name);
	bool getVisStatus() const;
	void setVisStatus(bool vis);

	std::shared_ptr<RobotVisualization> getVisualization();

	const boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> getSharedWorldInfo() const;
	void setSharedWorldInfo(const boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> msg);

        const boost::shared_ptr<msl_msgs::CorridorCheck>& getCorridorCheckInfo() const;
        void setCorridorCheckInfo(const boost::shared_ptr<msl_msgs::CorridorCheck>& corridorCheckInfo);

        const boost::shared_ptr<msl_msgs::PathPlanner>& getPathPlannerInfo() const;
        void setPathPlannerInfo(const boost::shared_ptr<msl_msgs::PathPlanner>& pathPlannerInfo);

        const boost::shared_ptr<msl_msgs::VoronoiNetInfo>& getVoronoiNetInfo() const;
        void setVoronoiNetInfo(const boost::shared_ptr<msl_msgs::VoronoiNetInfo>& voronoiNetInfo);

        const int getDebugMsgs(std::vector<boost::shared_ptr<msl_helper_msgs::DebugMsg>>& result);
        void addDebugMsg(const boost::shared_ptr<msl_helper_msgs::DebugMsg>& debugMsg);

        const boost::shared_ptr<msl_helper_msgs::PassMsg>& getPassMsg() const;
        void setPassMsg(const boost::shared_ptr<msl_helper_msgs::PassMsg>& passMsg);

	unsigned long getTimeStamp();
	void updateTimeStamp();

	bool isTimeout();
        bool isPathPlannerMsgTimeout();
        bool isVoronoiNetMsgTimeout();
        bool isCorridorCheckMsgTimeout();
        bool isPassMsgTimeout();

private:
   	bool myVis;
	int id;
	std::string name;
	unsigned long timeStamp;
        unsigned long pathPlannerMsgTimeStamp;
        unsigned long voronoiNetMsgTimeStamp;
        unsigned long corridorCheckMsgTimeStamp;
        unsigned long passMsgTimeStamp;
	std::shared_ptr<RobotVisualization> visualization;
	FieldWidget3D* field;

	boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> msg;
        boost::shared_ptr<msl_msgs::PathPlanner> pathPlannerInfo;
        boost::shared_ptr<msl_msgs::VoronoiNetInfo> voronoiNetInfo;
        boost::shared_ptr<msl_msgs::CorridorCheck> corridorCheckInfo;
        std::map<std::string,DebugMsg> debugMsg;
        boost::shared_ptr<msl_helper_msgs::PassMsg> passMsg;
};

#endif /* CNC_MSL_MSL_VISUALIZATION_SRC_TOOLS_BASESTATION_FIELDWIDGET_ROBOTINFO_H_ */
