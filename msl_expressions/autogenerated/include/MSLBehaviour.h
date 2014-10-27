/*
 * MSLBehaviour.h
 *
 *  Created on: 27.10.2014
 *      Author: endy
 */

#ifndef MSLBEHAVIOUR_H_
#define MSLBEHAVIOUR_H_

#include <iostream>
#include "engine/BasicBehaviour.h"
#include "ros/ros.h"
#include "msl_simulator/sim_packet.h"
#include "SystemConfig.h"

using namespace std;

namespace msl
{

	class MSLBehaviour : public alica::BasicBehaviour
	{
	public:
		MSLBehaviour(string name);
		virtual ~MSLBehaviour();
		void send(msl_simulator::sim_robot_command& p);

	private:
		int ownID;
		ros::Publisher simlatorPub;
	};

} /* namespace msl */

#endif /* MSLBEHAVIOUR_H_ */
