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
#include <string.h>


namespace msl_simulator
{

	MSLSimulatorROSCommunication::MSLSimulatorROSCommunication()
	{
		this->isRunning = false;
		this->isteamyellow = false;
		rosNode = new ros::NodeHandle();
		spinner = new ros::AsyncSpinner(4);

//		messagesRoboCupSSLWrapperPublisher = rosNode->advertise<messages_robocup_ssl_wrapper>("/MSLSimulator/MessagesRoboCupSSLWrapper", 2);

//		commandSubscriber = rosNode->subscribe("/MSLSimulator/SimPacket", 10, &MSLSimulatorROSCommunication::handleSimPacket, (MSLSimulatorROSCommunication*)this);

		motionControl = rosNode->subscribe("/MotionControl", 10, &MSLSimulatorROSCommunication::handleMotionControl, (MSLSimulatorROSCommunication*)this);

		ballInfoPublisher = rosNode->advertise<msl_sensor_msgs::BallInfo>(
		                                "/BallInfo", 10);

		worldModelPublisher = rosNode->advertise<msl_sensor_msgs::SimulatorWorldModelData>("/WorldModel/SimulatorWorldModelData", 10);

	}

	MSLSimulatorROSCommunication::~MSLSimulatorROSCommunication()
	{
		if (this->isRunning)
		{
			spinner->stop();
		}
		delete spinner;

//		messagesRoboCupSSLWrapperPublisher.shutdown();
		rosNode->shutdown();
		delete rosNode;

	}
	void MSLSimulatorROSCommunication::sendSimWorldData(msl_sensor_msgs::SimulatorWorldModelData simwm)
	{
		this->worldModelPublisher.publish(simwm);
	}

	void MSLSimulatorROSCommunication::sendBallInfoPtr(msl_sensor_msgs::BallInfo& ball)
	{
		this->ballInfoPublisher.publish(ball);
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
//		messagesRoboCupSSLWrapperPublisher.publish(packet);
	}

	void MSLSimulatorROSCommunication::handleMotionControl(msl_actuator_msgs::MotionControlPtr motion)
	{
		boost::lock_guard<boost::mutex> lock(mutex);
		recvQueue.push_back(motion);
	}

	msl_actuator_msgs::MotionControlPtr MSLSimulatorROSCommunication::getSimPacket()
	{
		boost::lock_guard<boost::mutex> lock(mutex);
		msl_actuator_msgs::MotionControlPtr motion = recvQueue.front();

		msl_simulator::sim_packetPtr packet;

		recvQueue.pop_front();
		return motion;
	}

	bool MSLSimulatorROSCommunication::isQueueEmpty()
	{
		boost::lock_guard<boost::mutex> lock(mutex);
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
