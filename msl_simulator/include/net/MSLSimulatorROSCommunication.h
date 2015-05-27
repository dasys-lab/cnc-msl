/*
 * MSLSimulatorROSCommunication.h
 *
 *  Created on: 22.10.2014
 *      Author: Stephan Opfer
 */

#ifndef MSLSIMULATORROSCOMMUNICATION_H_
#define MSLSIMULATORROSCOMMUNICATION_H_

#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <list>
#include "ros/ros.h"
#include <msl_sensor_msgs/BallInfo.h>
#include "msl_simulator/messages_robocup_ssl_wrapper.h"
#include "msl_simulator/sim_packet.h"
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_actuator_msgs/KickControl.h>
#include <msl_sensor_msgs/SimulatorWorldModelData.h>

using namespace std;

namespace msl_simulator
{

	class MSLSimulatorROSCommunication
	{
	public:
		MSLSimulatorROSCommunication();
		virtual ~MSLSimulatorROSCommunication();

		virtual void tick();

		virtual void send(messages_robocup_ssl_wrapperPtr packet);

		virtual void handleMotionControl(msl_actuator_msgs::MotionControlPtr);
		void handleKickControl(msl_actuator_msgs::KickControlPtr kick);
		void sendBallInfoPtr(msl_sensor_msgs::BallInfo& ball);
		void sendSimWorldData(msl_sensor_msgs::SimulatorWorldModelData simwm);

		virtual void startCommunication();
		virtual void stopCommunication();

		msl_actuator_msgs::MotionControlPtr getSimPacket();
		msl_actuator_msgs::KickControlPtr getKick();
		bool isQueueEmpty();

		bool isteamyellow;

	protected:
		ros::NodeHandle* rosNode;
		ros::AsyncSpinner* spinner;

		list<msl_actuator_msgs::MotionControlPtr> recvQueue;
		list<msl_actuator_msgs::KickControlPtr> kickQueue;
		boost::mutex mutex;
		boost::mutex kickmutex;

//		ros::Publisher messagesRoboCupSSLWrapperPublisher;
		ros::Publisher ballInfoPublisher;
		ros::Publisher worldModelPublisher;


		ros::Subscriber motionControl;
		ros::Subscriber kickControl;


		bool isRunning;
	};

} /* namespace msl_simulator */

#endif /* MSLSIMULATORROSCOMMUNICATION_H_ */
