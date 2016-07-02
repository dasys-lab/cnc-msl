#include "DomainBehaviour.h"

#include <RawSensorData.h>
#include <msl_robot/kicker/Kicker.h>
#include <SystemConfig.h>
#include <MSLWorldModel.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <msl_actuator_msgs/KickControl.h>
#include <msl_actuator_msgs/ShovelSelectCmd.h>
#include <msl_helper_msgs/DebugMsg.h>
#include <msl_helper_msgs/PassMsg.h>
#include <msl_helper_msgs/WatchBallMsg.h>
#include <msl_robot/MSLRobot.h>

namespace alica
{
	DomainBehaviour::DomainBehaviour(string name) :
			BasicBehaviour(name)
	{
		this->sc = supplementary::SystemConfig::getInstance();
		this->ownID = sc->getOwnRobotID();
		ros::NodeHandle n;
		this->wm = msl::MSLWorldModel::get();
		this->robot = msl::MSLRobot::get();

		if (wm->timeLastSimMsgReceived > 0)
		{
			motionControlPub = n.advertise<msl_actuator_msgs::MotionControl>(supplementary::SystemConfig::getHostname() + "/MotionControl", 10);
			ballHandlePub = n.advertise<msl_actuator_msgs::BallHandleCmd>(supplementary::SystemConfig::getHostname() + "/BallHandleControl", 10);
			kickControlPub = n.advertise<msl_actuator_msgs::KickControl>(supplementary::SystemConfig::getHostname() + "/KickControl", 10);
			shovelSelectPublisher = n.advertise<msl_actuator_msgs::ShovelSelectCmd>(supplementary::SystemConfig::getHostname() + "/ShovelSelectControl", 10);
		}
		else
		{
			motionControlPub = n.advertise<msl_actuator_msgs::MotionControl>("MotionControl", 10);
			ballHandlePub = n.advertise<msl_actuator_msgs::BallHandleCmd>("BallHandleControl", 10);
			kickControlPub = n.advertise<msl_actuator_msgs::KickControl>("KickControl", 10);
			shovelSelectPublisher = n.advertise<msl_actuator_msgs::ShovelSelectCmd>("ShovelSelectControl", 10);
		}
		passMsgPublisher = n.advertise<msl_helper_msgs::PassMsg>("WorldModel/PassMsg", 10);
		watchBallMsgPublisher = n.advertise<msl_helper_msgs::WatchBallMsg>("/WorldModel/WatchBallMsg", 10);
		debugMsgPublisher = n.advertise<msl_helper_msgs::DebugMsg>("/DebugMsg", 10);

		__maxTranslation = (*sc)["Globals"]->get<double>("Globals", "Team", sc->getHostname().c_str(), "AverageTranslation", NULL);
	}

	DomainBehaviour::~DomainBehaviour()
	{
	}

	void alica::DomainBehaviour::send(msl_actuator_msgs::MotionControl& mc)
	{
//        this->wm->prediction.monitoring();
		mc.senderID = ownID;
		mc.timestamp = wm->getTime();
		mc.motion.translation = min(__maxTranslation, mc.motion.translation);
		motionControlPub.publish(mc);
		wm->rawSensorData->processMotionControlMessage(mc);
	}

	void alica::DomainBehaviour::send(msl_actuator_msgs::BallHandleCmd& bh)
	{
		supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
		double minRotation = (*sys)["Actuation"]->get<double>("Dribble.MinRotation", NULL);
		bh.enabled = true;
		bh.senderID = ownID;
		// this is only for nase and his new left motor
		int sgnR = bh.rightMotor >= 0 ? 1 : -1;
		int sgnL = bh.leftMotor >= 0 ? 1 : -1;
		if (!(bh.rightMotor == 0) && !(bh.leftMotor == 0))
		{
			bh.rightMotor = (int)((max(minRotation, abs(bh.rightMotor * 1.0))) * sgnR / (*sys)["Actuation"]->get<double>("Dribble.DribbleFactorRight", NULL));
			bh.leftMotor = (int)((max(minRotation+400, abs(bh.rightMotor * 1.0))) * sgnL / (*sys)["Actuation"]->get<double>("Dribble.DribbleFactorLeft", NULL));
		}
		ballHandlePub.publish(bh);
		
	}

	void alica::DomainBehaviour::send(msl_actuator_msgs::KickControl& kc)
	{
		kc.enabled = true;
		kc.senderID = ownID;
		kickControlPub.publish(kc);
		kickControlPub.publish(kc);
		kickControlPub.publish(kc);
        this->robot->kicker->processKickConstrolMsg(kc);
	}

	void alica::DomainBehaviour::send(msl_actuator_msgs::ShovelSelectCmd& ssc)
	{
		ssc.senderID = ownID;
		shovelSelectPublisher.publish(ssc);
		this->robot->kicker->lowShovelSelected = ssc.passing;
	}

	void alica::DomainBehaviour::send(msl_helper_msgs::PassMsg& pm, int senderID)
	{
		pm.senderID = senderID;
		passMsgPublisher.publish(pm);
		passMsgPublisher.publish(pm);
	}

	void alica::DomainBehaviour::send(msl_helper_msgs::PassMsg& pm)
	{
		pm.senderID = ownID;
		passMsgPublisher.publish(pm);
		passMsgPublisher.publish(pm);
	}

	void alica::DomainBehaviour::send(msl_helper_msgs::WatchBallMsg& wb)
	{
		wb.senderID = ownID;
		watchBallMsgPublisher.publish(wb);
	}

	void alica::DomainBehaviour::send(msl_helper_msgs::DebugMsg& dbm)
	{
		dbm.senderID = ownID;
		debugMsgPublisher.publish(dbm);
	}
} /* namespace alica */

