#ifndef DomainBehaviour_H_
#define DomainBehaviour_H_

#include "engine/BasicBehaviour.h"
#include <iostream>
#include "ros/ros.h"
#include "msl_simulator/sim_packet.h"
#include "SystemConfig.h"
#include "MSLWorldModel.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "msl_actuator_msgs/BallHandleCmd.h"
#include "msl_actuator_msgs/KickControl.h"

namespace alica
{
class DomainBehaviour : public BasicBehaviour
{
	public:
		DomainBehaviour(string name);
		virtual ~DomainBehaviour();
		void send(msl_simulator::sim_robot_command& p);
		void send(msl_actuator_msgs::MotionControl& mc);
		void send(msl_actuator_msgs::BallHandleCmd& bh);
		void send(msl_actuator_msgs::KickControl& kc);
		msl::MSLWorldModel* wm;

	protected:
		supplementary::SystemConfig* sc;

	private:

		int ownID;
		ros::Publisher simlatorPub;
		ros::Publisher motionControlPub;
		ros::Publisher ballHandlePub;
		ros::Publisher kickControlPub;
	};
} /* namespace alica */

#endif /* DomainBehaviour_H_ */

