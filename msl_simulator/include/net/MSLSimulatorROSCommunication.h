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

#include "msl_simulator/messages_robocup_ssl_wrapper.h"
#include "msl_simulator/sim_packet.h"

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

		virtual void handleSimPacket(sim_packetPtr simPacket);

		virtual void startCommunication();
		virtual void stopCommunication();

		sim_packetPtr getSimPacket();
		bool isQueueEmpty();

	protected:
		ros::NodeHandle* rosNode;
		ros::AsyncSpinner* spinner;

		list<sim_packetPtr> recvQueue;
		boost::mutex mutex;

		ros::Publisher messagesRoboCupSSLWrapperPublisher;
		ros::Subscriber commandSubscriber;

		bool isRunning;
	};

} /* namespace msl_simulator */

#endif /* MSLSIMULATORROSCOMMUNICATION_H_ */
