#include "DomainBehaviour.h"

#include <Ball.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <SystemConfig.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <msl_actuator_msgs/KickControl.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_actuator_msgs/ShovelSelectCmd.h>
#include <msl_helper_msgs/DebugMsg.h>
#include <msl_helper_msgs/PassMsg.h>
#include <msl_helper_msgs/WatchBallMsg.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/kicker/Kicker.h>
#include <msl_robot/robotmovement/RobotMovement.h>

namespace alica
{
DomainBehaviour::DomainBehaviour(string name)
    : BasicBehaviour(name)
{
    this->sc = supplementary::SystemConfig::getInstance();
    this->ownID = sc->getOwnRobotID();
    ros::NodeHandle n;
    this->wm = msl::MSLWorldModel::get();
    this->robot = msl::MSLRobot::get();
      
		if (this->wm->isUsingSimulator())
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

    this->__maxTranslation = (*sc)["Drive"]->get<double>("Drive", "MaxSpeed", NULL);
    this->__maxTranslationWithoutBall = (*sc)["Drive"]->get<double>("Drive", "MaxSpeedWithoutBall", NULL);
    this->minRotation = (*sc)["Actuation"]->get<double>("Dribble.MinRotation", NULL);
    this->minRotationLeft = (*sc)["Actuation"]->get<double>("Dribble.MinRotationLeft", NULL);
    this->minRotationRight = (*sc)["Actuation"]->get<double>("Dribble.MinRotationRight", NULL);
    this->dribbleFactorRight = (*sc)["Actuation"]->get<double>("Dribble.DribbleFactorRight", NULL);
    this->dribbleFactorLeft = (*sc)["Actuation"]->get<double>("Dribble.DribbleFactorLeft", NULL);
}

DomainBehaviour::~DomainBehaviour()
{
}

/**
 * Sends a MotionControl
 * Only use this method when using methods from robot movement
 * If you are using a behaviour specific controller use DomainBehaviour::sendAndUpdatePT(msl_actuator_msgs::MotionControl &mc)
 */
void alica::DomainBehaviour::send(msl_actuator_msgs::MotionControl &mc)
{
    //        this->wm->prediction.monitoring();
    mc.senderID = ownID;
    mc.timestamp = wm->getTime();
    if(wm->ball->haveBall())
    {
    	mc.motion.translation = min(__maxTranslation, mc.motion.translation);
    }
    else
    {
    	mc.motion.translation = min(this->__maxTranslationWithoutBall, mc.motion.translation);
    }
    motionControlPub.publish(mc);
    wm->rawSensorData->processMotionControlMessage(mc);
}

/**
 * Sends a MotionControl and updates the central PT controller to the corresponding values
 * Only use this method when using a behaviour specific controller in order to keep the PT controller up to date
 * If you are using the RobotMovement use DomainBehaviour::send(msl_actuator_msgs::MotionControl &mc)
 */
void alica::DomainBehaviour::sendAndUpdatePT(msl_actuator_msgs::MotionControl &mc)
{
    mc.senderID = ownID;
    mc.timestamp = wm->getTime();
    if(wm->ball->haveBall())
    {
    	mc.motion.translation = min(__maxTranslation, mc.motion.translation);
    }
    else
    {
    	mc.motion.translation = min(this->__maxTranslationWithoutBall, mc.motion.translation);
    }
    motionControlPub.publish(mc);
    wm->rawSensorData->processMotionControlMessage(mc);
    robot->robotMovement->updatePT();
}

void alica::DomainBehaviour::send(msl_actuator_msgs::BallHandleCmd &bh)
{
    bh.enabled = true;
    bh.senderID = ownID;
    // this was written for nase and his new left motor
    //this should not be necessary anymore, because we collected the current->velocity splines for each motor separately
//    if (bh.rightMotor != 0 && bh.leftMotor != 0)
//    {
//        bh.rightMotor = (int)(max(this->minRotationRight, (abs(bh.rightMotor)) / this->dribbleFactorRight)) * (bh.rightMotor >= 0 ? 1 : -1);
//        bh.leftMotor = (int)(max(this->minRotationLeft, (abs(bh.leftMotor)) / this->dribbleFactorLeft)) * (bh.leftMotor >= 0 ? 1 : -1);
//    }
        if (bh.rightMotor != 0 && bh.leftMotor != 0)
        {
            bh.rightMotor = (int)(max(this->minRotationRight, (double)(abs(bh.rightMotor)))) * (bh.rightMotor >= 0 ? 1 : -1);
            bh.leftMotor = (int)(max(this->minRotationLeft, (double)(abs(bh.leftMotor)))) * (bh.leftMotor >= 0 ? 1 : -1);
        }
    this->ballHandlePub.publish(bh);
}

void alica::DomainBehaviour::send(msl_actuator_msgs::KickControl &kc)
{
    kc.enabled = true;
    kc.senderID = ownID;
    this->kickControlPub.publish(kc);
    this->kickControlPub.publish(kc);
    this->kickControlPub.publish(kc);
    this->robot->kicker->processKickConstrolMsg(kc);
}

void alica::DomainBehaviour::send(msl_actuator_msgs::ShovelSelectCmd &ssc)
{
    ssc.senderID = ownID;
    this->shovelSelectPublisher.publish(ssc);
    this->robot->kicker->lowShovelSelected = ssc.passing;
}

void alica::DomainBehaviour::send(msl_helper_msgs::PassMsg &pm, int senderID)
{
    pm.senderID = senderID;
    this->passMsgPublisher.publish(pm);
    this->passMsgPublisher.publish(pm);
}

void alica::DomainBehaviour::send(msl_helper_msgs::PassMsg &pm)
{
    pm.senderID = ownID;
    this->passMsgPublisher.publish(pm);
    this->passMsgPublisher.publish(pm);
}

void alica::DomainBehaviour::send(msl_helper_msgs::WatchBallMsg &wb)
{
    wb.senderID = ownID;
    this->watchBallMsgPublisher.publish(wb);
}

void alica::DomainBehaviour::send(msl_helper_msgs::DebugMsg &dbm)
{
    dbm.senderID = ownID;
    debugMsgPublisher.publish(dbm);
}
} /* namespace alica */
