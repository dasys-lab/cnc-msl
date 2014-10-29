#include "DomainBehaviour.h"

namespace alica
{
DomainBehaviour::DomainBehaviour(string name) :
    BasicBehaviour(name)
{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		ownID = sc->getOwnRobotID();
		ros::NodeHandle n;
		simlatorPub = n.advertise<msl_simulator::sim_packet>("/MSLSimulator/SimPacket", 1000);
}

DomainBehaviour::~DomainBehaviour()
{
}
void DomainBehaviour::send(msl_simulator::sim_robot_command& p)
	{
		p.id = ownID;
		msl_simulator::sim_packet pa;
		pa.commands.isteamyellow = true;
		pa.commands.robot_commands.push_back(p);
		simlatorPub.publish(pa);
	}

} /* namespace alica */
