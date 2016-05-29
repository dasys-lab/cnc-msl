#include "DomainBehaviour.h"
#include <engine/RunningPlan.h>
#include <SystemConfig.h>
#include <MSLWorldModel.h>

namespace alica
{
	DomainBehaviour::DomainBehaviour(string name) : BasicBehaviour(name)
	{
		this->sc = supplementary::SystemConfig::getInstance();
		this->ownID = sc->getOwnRobotID();
		this->wm = msl::MSLWorldModel::get();

		ros::NodeHandle n;
		if (this->wm->timeLastSimMsgReceived > 0)
		{
			this->motionControlPub = n.advertise<msl_actuator_msgs::MotionControl>(
					supplementary::SystemConfig::getHostname() + "/MotionControl", 10);
			this->ballHandlePub = n.advertise<msl_actuator_msgs::BallHandleCmd>(
					supplementary::SystemConfig::getHostname() + "/BallHandleControl", 10);
			this->kickControlPub = n.advertise<msl_actuator_msgs::KickControl>(
					supplementary::SystemConfig::getHostname() + "/KickControl", 10);
			this->shovelSelectPublisher = n.advertise<msl_actuator_msgs::ShovelSelectCmd>(
					supplementary::SystemConfig::getHostname() + "/ShovelSelectControl", 10);
		}
		else
		{
			this->motionControlPub = n.advertise<msl_actuator_msgs::MotionControl>("MotionControl", 10);
			this->ballHandlePub = n.advertise<msl_actuator_msgs::BallHandleCmd>("BallHandleControl", 10);
			this->kickControlPub = n.advertise<msl_actuator_msgs::KickControl>("KickControl", 10);
			this->shovelSelectPublisher = n.advertise<msl_actuator_msgs::ShovelSelectCmd>("ShovelSelectControl", 10);
		}
		this->passMsgPublisher = n.advertise<msl_helper_msgs::PassMsg>("WorldModel/PassMsg", 10);
		this->watchBallMsgPublisher = n.advertise<msl_helper_msgs::WatchBallMsg>("/WorldModel/WatchBallMsg", 10);
		this->debugMsgPublisher = n.advertise<msl_helper_msgs::DebugMsg>("/DebugMsg", 10);

		this->__maxTranslation = (*sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
	}

	DomainBehaviour::~DomainBehaviour()
	{
	}

	void DomainBehaviour::send(msl_actuator_msgs::MotionControl& mc)
	{
//      this->wm->prediction.monitoring();
		mc.senderID = ownID;
		mc.timestamp = wm->getTime();
		mc.motion.translation = min(__maxTranslation, mc.motion.translation);
		this->motionControlPub.publish(mc);
		this->wm->rawSensorData->processMotionControlMessage(mc);
	}

	void DomainBehaviour::send(msl_actuator_msgs::BallHandleCmd& bh)
	{
		bh.enabled = true;
		bh.senderID = ownID;
		this->ballHandlePub.publish(bh);
	}

	void DomainBehaviour::send(msl_actuator_msgs::KickControl& kc)
	{
		kc.enabled = true;
		kc.senderID = ownID;
		this->kickControlPub.publish(kc);
		this->kickControlPub.publish(kc);
		this->kickControlPub.publish(kc);
		this->wm->kicker->processKickConstrolMsg(kc);
	}

	void DomainBehaviour::send(msl_actuator_msgs::ShovelSelectCmd& ssc)
	{
		ssc.senderID = ownID;
		this->shovelSelectPublisher.publish(ssc);
		this->wm->kicker->lowShovelSelected = ssc.passing;
	}

	void DomainBehaviour::send(msl_helper_msgs::PassMsg& pm, int senderID)
	{
		pm.senderID = senderID;
		this->passMsgPublisher.publish(pm);
		this->passMsgPublisher.publish(pm);
	}

	void DomainBehaviour::send(msl_helper_msgs::PassMsg& pm)
	{
		send(pm, ownID);
	}

	void DomainBehaviour::send(msl_helper_msgs::WatchBallMsg& wb)
	{
		wb.senderID = ownID;
		this->watchBallMsgPublisher.publish(wb);
	}

	void DomainBehaviour::send(msl_helper_msgs::DebugMsg& dbm)
	{
		dbm.senderID = ownID;
		this->debugMsgPublisher.publish(dbm);
	}

	/**
	 * Returns the current position of the first teammate,
	 * allocated to the given EntryPoint.
	 * @param EntryPoint* ep
	 * @return Position of the teammate, or nullptr.
	 */
	shared_ptr<geometry::CNPosition> DomainBehaviour::getTeammatesPosition(EntryPoint* ep)
	{
		// get the plan in which the behavior is running
		auto parent = this->runningPlan->getParent().lock();
		if (parent == nullptr)
		{
			return nullptr;
		}
		// get robot ids of robots in found entry point
		shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep);
		// exactly one robot is receiver
		if (ids->empty() || ids->at(0) == -1)
		{
			return nullptr;
		}
		// get receiver position by id
		return this->wm->robots->teammates.getTeamMatePosition(ids->at(0));
	}
} /* namespace alica */

