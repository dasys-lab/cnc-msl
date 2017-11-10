#include "DomainBehaviour.h"

#include <msl_actuator_msgs/BallHandleCmd.h>
#include <msl_actuator_msgs/KickControl.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_actuator_msgs/ShovelSelectCmd.h>
#include <msl_helper_msgs/DebugMsg.h>
#include <msl_helper_msgs/PassMsg.h>
#include <msl_helper_msgs/WatchBallMsg.h>
#include <msl_robot/MSLRobot.h>

#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <SystemConfig.h>
#include <msl_robot/kicker/Kicker.h>
#include <msl/robot/IntRobotID.h>
#include <engine/AlicaEngine.h>

using std::string;
namespace alica
{
DomainBehaviour::DomainBehaviour(string name)
    : BasicBehaviour(name)
	, robot(nullptr)
	, ownID(nullptr)
{
    this->wm = msl::MSLWorldModel::get();
    this->sc = supplementary::SystemConfig::getInstance();
    passMsgPublisher = n.advertise<msl_helper_msgs::PassMsg>("WorldModel/PassMsg", 10);
    watchBallMsgPublisher = n.advertise<msl_helper_msgs::WatchBallMsg>("/WorldModel/WatchBallMsg", 10);
    debugMsgPublisher = n.advertise<msl_helper_msgs::DebugMsg>("/DebugMsg", 10);

    __maxTranslation = (*sc)["Globals"]->get<double>("Globals", "Team", sc->getHostname().c_str(), "AverageTranslation", NULL);
}

DomainBehaviour::~DomainBehaviour()
{
}

void alica::DomainBehaviour::send(msl_actuator_msgs::MotionControl &mc)
{
    //        this->wm->prediction.monitoring();
    mc.timestamp = wm->getTime();
    mc.motion.translation = min(__maxTranslation, mc.motion.translation);
    motionControlPub.publish(mc);
    wm->rawSensorData->processMotionControlMessage(mc);
}

void alica::DomainBehaviour::send(msl_actuator_msgs::BallHandleCmd &bh)
{
    supplementary::SystemConfig *sys = supplementary::SystemConfig::getInstance();
    double minRotation = (*sys)["Actuation"]->get<double>("Dribble.MinRotation", NULL);
    double minRotationLeft = (*sys)["Actuation"]->get<double>("Dribble.MinRotationLeft", NULL);
    double minRotationRight = (*sys)["Actuation"]->get<double>("Dribble.MinRotationRight", NULL);
    bh.enabled = true;
    bh.senderID.id = ownID->toByteVector();
    // this is only for nase and his new left motor
    int sgnR = bh.rightMotor >= 0 ? 1 : -1;
    int sgnL = bh.leftMotor >= 0 ? 1 : -1;
    if (!(bh.rightMotor == 0) && !(bh.leftMotor == 0))
    {
        bh.rightMotor = (int)(max(minRotationRight, (abs(bh.rightMotor * 1.0)) / (*sys)["Actuation"]->get<double>("Dribble.DribbleFactorRight", NULL))) * sgnR;
        bh.leftMotor = (int)(max(minRotationLeft, (abs(bh.leftMotor * 1.0)) / (*sys)["Actuation"]->get<double>("Dribble.DribbleFactorLeft", NULL))) * sgnL;
    }
    ballHandlePub.publish(bh);
}

void alica::DomainBehaviour::send(msl_actuator_msgs::KickControl &kc)
{
    kc.enabled = true;
    kickControlPub.publish(kc);
    kickControlPub.publish(kc);
    kickControlPub.publish(kc);
    this->robot->kicker->processKickConstrolMsg(kc);
}

void alica::DomainBehaviour::send(msl_actuator_msgs::ShovelSelectCmd &ssc)
{
    shovelSelectPublisher.publish(ssc);
    this->robot->kicker->lowShovelSelected = ssc.passing;
}

void alica::DomainBehaviour::send(msl_helper_msgs::PassMsg &pm, std::vector<uint8_t> senderID)
{
    pm.senderID.id = senderID;
    passMsgPublisher.publish(pm);
    passMsgPublisher.publish(pm);
}

void alica::DomainBehaviour::send(msl_helper_msgs::PassMsg &pm)
{
    pm.senderID.id = ownID->toByteVector();
    passMsgPublisher.publish(pm);
    passMsgPublisher.publish(pm);
}

void alica::DomainBehaviour::send(msl_helper_msgs::WatchBallMsg &wb)
{
    wb.senderID.id = ownID->toByteVector();
    watchBallMsgPublisher.publish(wb);
}

void alica::DomainBehaviour::send(msl_helper_msgs::DebugMsg &dbm)
{
    dbm.senderID.id = ownID->toByteVector();
    debugMsgPublisher.publish(dbm);
}

void DomainBehaviour::init()
{
	auto tmp = sc->getOwnRobotID();
	    this->ownID = dynamic_cast<const msl::robot::IntRobotID*>(this->getOwnId());
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
}

} /* namespace alica */
