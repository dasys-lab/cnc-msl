/*
 * MSLBehaviour.cpp
 *
 *  Created on: 27.10.2014
 *      Author: endy
 */

#include "MSLBehaviour.h"

namespace msl
{

	MSLBehaviour::MSLBehaviour(string name) : alica::BasicBehaviour(name)
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		ownID = sc->getOwnRobotID();
		ros::NodeHandle n;
		simlatorPub = n.advertise<msl_simulator::sim_packet>("/MSLSimulator/SimPacket", 1000);
	}

	MSLBehaviour::~MSLBehaviour()
	{
	}

	void MSLBehaviour::send(msl_simulator::sim_robot_command& p)
	{
		p.id = ownID;
		msl_simulator::sim_packet pa;
		pa.commands.isteamyellow = true;
		pa.commands.robot_commands.push_back(p);
		simlatorPub.publish(pa);
	}

} /* namespace msl */
