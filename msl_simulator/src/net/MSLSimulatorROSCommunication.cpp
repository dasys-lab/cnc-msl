/*
 * MSLSimulatorROSCommunication.cpp
 *
 *  Created on: 22.10.2014
 *      Author: Stephan Opfer
 */

#include "net/MSLSimulatorROSCommunication.h"

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace msl_simulator
{

	MSLSimulatorROSCommunication::MSLSimulatorROSCommunication()
	{
		this->isRunning = false;
		rosNode = new ros::NodeHandle();
		spinner = new ros::AsyncSpinner(4);

		messagesRoboCupSSLWrapperPublisher = rosNode->advertise<messages_robocup_ssl_wrapper>("/MSLSimulator/MessagesRoboCupSSLWrapper", 2);

		commandSubscriber = rosNode->subscribe("/MSLSimulator/SimPacket", 10, &MSLSimulatorROSCommunication::handleSimPacket, (MSLSimulatorROSCommunication*)this);
	}

	MSLSimulatorROSCommunication::~MSLSimulatorROSCommunication()
	{
		if (this->isRunning)
		{
			spinner->stop();
		}
		delete spinner;

		messagesRoboCupSSLWrapperPublisher.shutdown();
		rosNode->shutdown();
		delete rosNode;

	}

	void MSLSimulatorROSCommunication::tick()
	{
		if (this->isRunning)
		{
			ros::spinOnce();
		}
	}

	void MSLSimulatorROSCommunication::send(messages_robocup_ssl_wrapperPtr packet)
	{
		messagesRoboCupSSLWrapperPublisher.publish(packet);
	}

	void MSLSimulatorROSCommunication::handleSimPacket(sim_packetPtr simPacket)
	{
		boost::lock_guard<boost::mutex> lock(mutex);
		recvQueue.push_back(simPacket);
	}

	sim_packetPtr MSLSimulatorROSCommunication::getSimPacket()
	{
		boost::lock_guard<boost::mutex> lock(mutex);
		sim_packetPtr packet = recvQueue.front();
		recvQueue.pop_front();
		return packet;
	}

	bool MSLSimulatorROSCommunication::isQueueEmpty()
	{
		boost::lock_guard<boost::mutex> lock(mutex);
		cout << recvQueue.size() << " " <<recvQueue.empty() << endl;
		return recvQueue.empty();
	}

	void MSLSimulatorROSCommunication::startCommunication()
	{
		this->isRunning = true;
		spinner->start();
	}
	void MSLSimulatorROSCommunication::stopCommunication()
	{
		this->isRunning = false;
		spinner->stop();
	}

} /* namespace alicaRosProxy */
