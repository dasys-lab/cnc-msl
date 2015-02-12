/*
 * RobotInfo.h
 *
 *  Created on: Feb 12, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_VISUALIZATION_SRC_TOOLS_BASESTATION_FIELDWIDGET_ROBOTINFO_H_
#define CNC_MSL_MSL_VISUALIZATION_SRC_TOOLS_BASESTATION_FIELDWIDGET_ROBOTINFO_H_

#include <boost/shared_ptr.hpp>
#include "msl_sensor_msgs/SharedWorldInfo.h"
#include <ros/ros.h>

class RobotInfo
{
public:
	RobotInfo();
	virtual ~RobotInfo();
	int getId() const;
	void setId(int id);
	boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> getMsg();
	void setMsg(boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> msg);
	unsigned long getTimeStamp();
	void setTimeStamp(unsigned long timeStamp);

private:
	int id;
	unsigned long timeStamp;
	boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> msg;
};

#endif /* CNC_MSL_MSL_VISUALIZATION_SRC_TOOLS_BASESTATION_FIELDWIDGET_ROBOTINFO_H_ */
