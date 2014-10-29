/*
 * flooding_test_node.cpp
 *
 *  Created on: 27.10.2014
 *      Author: endy
 */

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "flooding_test/TestMessage.h"
#include "CaceMultiCastChannel.h"

#include <sstream>

using namespace std;
using namespace multicast;

int ownID = 0;

class MultiCastReceive {
public:
	MultiCastReceive() {}
	~MultiCastReceive() {}

	void callback(char* buffer, int size) {
		cout << size << endl;
	}
};

MultiCastChannel<MultiCastReceive>* commandChannel;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void messageCallback(flooding_test::TestMessagePtr msg)
{
	commandChannel->publish("asd", 3);
	//cout << "I heard: " << to_string(msg->id) << " from NodeID: " << msg->sender << endl;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

	if (argc < 2)
	{
		cout << "Usage: [Executable] [Node_ID]" << endl;
		return 0;
	}
	ownID = std::atoi(argv[1]);
	cout << "Node ID is: " << ownID << endl;

	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "flooding_test_node");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	ros::Publisher chatter_pub = n.advertise<flooding_test::TestMessage>("flooding_test/TestMessage", 10);

	ros::Rate loop_rate(10);

	/**
	 * The subscribe() call is how you tell ROS that you want to receive messages
	 * on a given topic.  This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing.  Messages are passed to a callback function, here
	 * called messageCallback.  subscribe() returns a Subscriber object that you
	 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	 * object go out of scope, this callback will automatically be unsubscribed from
	 * this topic.
	 *
	 * The second parameter to the subscribe() function is the size of the message
	 * queue.  If messages are arriving faster than they are being processed, this
	 * is the number of messages that will be buffered up before beginning to throw
	 * away the oldest ones.
	 */
	ros::Subscriber sub = n.subscribe("flooding_test/TestMessage", 30, messageCallback);

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	int count = 0;

	if (ownID == 3)
	{
		cout << "send a message" << endl;
	}

	MultiCastReceive mcr;
	string addr = "224.16.32.40";
    commandChannel = new MultiCastChannel<MultiCastReceive>(
			addr, 50000, &MultiCastReceive::callback, &mcr);

	while (ros::ok())
	{
		ros::spinOnce();
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */
		flooding_test::TestMessage msg;

		msg.id = count;
		msg.sender = ownID;

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		cout << "Sending: " << msg.id << endl;
		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}
