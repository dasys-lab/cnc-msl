#include "DomainBehaviour.h"

namespace alica
{
	msl::MSLWorldModel* wm;

	DomainBehaviour::DomainBehaviour(string name) :
			BasicBehaviour(name)
	{
		sc = supplementary::SystemConfig::getInstance();
		ownID = sc->getOwnRobotID();
		ros::NodeHandle n;
		wm = msl::MSLWorldModel::get();

		motionControlPub = n.advertise<msl_actuator_msgs::MotionControl>("/MotionControl", 10);

		ballHandlePub = n.advertise<msl_actuator_msgs::BallHandleCmd>("/BallHandleControl", 10);

		kickControlPub = n.advertise<msl_actuator_msgs::KickControl>("/KickControl", 10);
	}

	DomainBehaviour::~DomainBehaviour()
	{
	}

	void alica::DomainBehaviour::send(msl_actuator_msgs::MotionControl& mc)
	{
		mc.senderID = ownID;
		motionControlPub.publish(mc);
	}

	void alica::DomainBehaviour::send(msl_actuator_msgs::BallHandleCmd& bh)
	{
		bh.enabled = true;
		bh.senderID = ownID;
		ballHandlePub.publish(bh);

	}

	void alica::DomainBehaviour::send(msl_actuator_msgs::KickControl& kc)
	{
		kc.enabled = true;
		kc.senderID = ownID;
		kickControlPub.publish(kc);
		kickControlPub.publish(kc);
		kickControlPub.publish(kc);
	}
} /* namespace alica */

