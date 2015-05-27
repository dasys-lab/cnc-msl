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

		  kickControl = rosNode->subscribe("/KickControl", 10, &MSLSimulatorROSCommunication::handleKickControl, (MSLSimulatorROSCommunication*)this);


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
	void MSLSimulatorROSCommunication::handleKickControl(msl_actuator_msgs::KickControlPtr kick)
	{
		boost::lock_guard<boost::mutex> lock(kickmutex);
		kickQueue.push_back(kick);
	}

	msl_actuator_msgs::MotionControlPtr MSLSimulatorROSCommunication::getSimPacket()
	{
		boost::lock_guard<boost::mutex> lock(mutex);
		msl_actuator_msgs::MotionControlPtr motion = recvQueue.front();

		recvQueue.pop_front();
		return motion;
	}
	msl_actuator_msgs::KickControlPtr MSLSimulatorROSCommunication::getKick()
	{
		boost::lock_guard<boost::mutex> lock(kickmutex);
		msl_actuator_msgs::KickControlPtr kick;
		if(kickQueue.size() != 0)
		{
			kick = kickQueue.front();

			kickQueue.pop_front();
			return kick;
		}
		msl_actuator_msgs::KickControl kickToPtr;
		kickToPtr.senderID =  0;
		kick = boost::make_shared<msl_actuator_msgs::KickControl>(kickToPtr);
		return kick;
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
