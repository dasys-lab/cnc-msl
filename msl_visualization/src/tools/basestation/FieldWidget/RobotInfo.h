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
#include <memory>
#include <vector>

#include <msl_sensor_msgs/SharedWorldInfo.h>
#include <msl_helper_msgs/DebugMsg.h>
#include <msl_msgs/CorridorCheck.h>
#include <msl_msgs/PathPlanner.h>
#include <msl_msgs/VoronoiNetInfo.h>

#include "src/tools/basestation/RobotVisualization/RobotVisualization.h"

class FieldWidget3D;

class RobotInfo
{
public:
	RobotInfo(FieldWidget3D* field);
	virtual ~RobotInfo();
	int getId() const;
	void setId(int id);
	std::shared_ptr<RobotVisualization> getVisualization();
	const boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> getSharedWorldInfo() const;
	void setSharedWorldInfo(const boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> msg);
        const boost::shared_ptr<msl_msgs::CorridorCheck>& getCorridorCheckInfo() const;
        void setCorridorCheckInfo(const boost::shared_ptr<msl_msgs::CorridorCheck>& corridorCheckInfo);
        const boost::shared_ptr<msl_msgs::PathPlanner>& getPathPlannerInfo() const;
        void setPathPlannerInfo(const boost::shared_ptr<msl_msgs::PathPlanner>& pathPlannerInfo);
        const boost::shared_ptr<msl_msgs::VoronoiNetInfo>& getVoronoiNetInfo() const;
        void setVoronoiNetInfo(const boost::shared_ptr<msl_msgs::VoronoiNetInfo>& voronoiNetInfo);
	unsigned long getTimeStamp();
	void setTimeStamp(unsigned long timeStamp);
	void updateTimeStamp();

	bool isTimeout();

private:
	int id = 0;
	unsigned long timeStamp = 0;
	std::shared_ptr<RobotVisualization> visualization;
	FieldWidget3D* field;

	boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> msg = nullptr;
        boost::shared_ptr<msl_msgs::PathPlanner> pathPlannerInfo = nullptr;
        boost::shared_ptr<msl_msgs::VoronoiNetInfo> voronoiNetInfo = nullptr;
        boost::shared_ptr<msl_msgs::CorridorCheck> corridorCheckInfo = nullptr;
        boost::shared_ptr<msl_helper_msgs::DebugMsg> corridorCheckInfo = nullptr;
};

#endif /* CNC_MSL_MSL_VISUALIZATION_SRC_TOOLS_BASESTATION_FIELDWIDGET_ROBOTINFO_H_ */
