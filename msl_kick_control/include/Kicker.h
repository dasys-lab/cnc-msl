/*
 * Kicker.h
 *
 *  Created on: Nov 13, 2014
 *      Author: Stephan Opfer
 */

#ifndef KICKER_H_
#define KICKER_H_

#include <string>

#include "ros/ros.h"
#include <boost/thread/shared_mutex.hpp>

#include "msl_actuator_msgs/CanMsg.h"
#include "msl_actuator_msgs/KickControl.h"
#include "msl_actuator_msgs/KickTime.h"
#include "msl_actuator_msgs/KickerStatInfo.h"

#include <Can.h>
#include <SystemConfig.h>

namespace msldriver
{

	class Kicker
	{

		enum Cmd
		{
			Ping = 0x01, Rotate = 0x02, Kick = 0x03, Manual = 0x6D, SetDebugLevel = 0x10, SetPulseWidth = 0x30,
			SetMaxVoltage = 0x31, GetState = 0x40, GetVersion = 0x41, Pong = 0xF1, State = 0xF2, Version = 0xF3,
			Reset = 0xF4, Error = 0x21, Warning = 0x22, Msg = 0x3E,
		};

	public:
		Kicker();
		~Kicker();
		void setMaxVoltage();
		void onKickCmd(const msl_actuator_msgs::KickControl ks);
		void onKickerMsg(const msl_actuator_msgs::CanMsg message);
		string msg2string(vector<uint8_t> data);
		void setLastBeat();
		void sendHeartBeat(ros::Time now);
		void sendKickTime(ros::Time now);
		void checkStatus(ros::Time now);
		void sendStatus(ros::Time now);
		struct settings
		{
			bool highResolution;
			int alivePeriod;
			int maxVoltage;
			int robotId;
		} settings;

	private:
		ros::NodeHandle* rosNode;
		ros::Subscriber sub;
		ros::Subscriber kickSub;
		ros::Publisher rekickPub;
		ros::Publisher kickerTime;
		ros::Publisher kickerStat;

		ros::Time lastBeat;
		ros::Time lastKickRequest;

		ros::Time lastStatInfo;
		ros::Time lastKickTimeInfo;

		ros::Time lastKickStatUpdate;

		int lastPower;
		double lastCapacitorsVoltage;
		double lastSupplyVoltage;

		string rekickmessage;
		string rekickwarn;
		string rekickerror;

		//in sec
		double heartbeat_timeout = 1.5f;
	};
}

#endif /* KICKER_H_ */
