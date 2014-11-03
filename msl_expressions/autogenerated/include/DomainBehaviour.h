#ifndef DomainBehaviour_H_
#define DomainBehaviour_H_

#include "engine/BasicBehaviour.h"
#include <iostream>
#include "ros/ros.h"
#include "msl_simulator/sim_packet.h"
#include "SystemConfig.h"
#include "MSLWorldModel.h"
namespace alica
{
class DomainBehaviour : public BasicBehaviour
{
	public:
		DomainBehaviour(string name);
		virtual ~DomainBehaviour();
		void send(msl_simulator::sim_robot_command& p);
		msl::MSLWorldModel* wm;

	private:

		int ownID;
		ros::Publisher simlatorPub;
	};
} /* namespace alica */

#endif /* DomainBehaviour_H_ */

