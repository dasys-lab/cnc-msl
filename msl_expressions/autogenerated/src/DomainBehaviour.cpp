#include "DomainBehaviour.h"

namespace alica {
	msl::MSLWorldModel* wm;

	DomainBehaviour::DomainBehaviour(string name) :
			BasicBehaviour(name) {
		sc = supplementary::SystemConfig::getInstance();
		ownID = sc->getOwnRobotID();
		ros::NodeHandle n;
		simlatorPub = n.advertise<msl_simulator::sim_packet>(
				"/MSLSimulator/SimPacket", 1000);
		wm = msl::MSLWorldModel::get();

		motionControlPub = n.advertise<msl_actuator_msgs::MotionControl>(
				"/MotionControl", 10);

		ballHandlePub = n.advertise<msl_actuator_msgs::BallHandleCmd>(
				"/BallHandleControl", 10);

		kickControlPub = n.advertise<msl_actuator_msgs::KickControl>(
				"/KickControl", 10);
	}

	DomainBehaviour::~DomainBehaviour() {
	}

	void DomainBehaviour::send(msl_simulator::sim_robot_command& p) {
		p.id = ownID;
		msl_simulator::sim_packet pa;
		pa.commands.isteamyellow = true;
		pa.commands.robot_commands.push_back(p);
		simlatorPub.publish(pa);
	}

	void alica::DomainBehaviour::send(msl_actuator_msgs::MotionControl& mc) {
		mc.senderID = ownID;
		motionControlPub.publish(mc);
	}

	void alica::DomainBehaviour::send(msl_actuator_msgs::BallHandleCmd& bh) {
		bh.enabled = true;
		bh.senderID = ownID;
		ballHandlePub.publish(bh);

	}

	void alica::DomainBehaviour::send(msl_actuator_msgs::KickControl& kc) {
		kc.enabled = true;
		kc.senderID = ownID;
		kickControlPub.publish(kc);
	}
} /* namespace alica */

