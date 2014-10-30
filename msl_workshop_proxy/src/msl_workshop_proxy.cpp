/*
 * flooding_test_node.cpp
 *
 *  Created on: 27.10.2014
 *      Author: endy
 */

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "msl_workshop_proxy/TestMessage.h"
#include "CaceMultiCastChannel.h"

#include <sstream>

using namespace std;
using namespace multicast;

int ownID = 0;

const int mixed_team_flag_size = 1;
const int ball_size = 36;
const int opp_size = 4;
const int opp_count = 10;
const int position_size = 4;



struct ballPos
{
	int16_t ballX;
	int16_t ballY;
	int16_t ballZ;
	int16_t ballVX;
	int16_t ballVY;
	int16_t ballVZ;
	void append(unsigned char* ptr)
	{
		int16_t arr[] = {ballX, ballY, ballZ, ballVX, ballVY, ballVZ};
		unsigned char* it = (unsigned char*)&arr[0];
		for (int i = 0; i < ball_size; i++)
		{
			ptr[i] = it[i];
		}
	}

	void desrializeFromPtr(unsigned char* ptr)
	{
		int16_t* it = (int16_t*)&ptr[0];
		ballX = *it;
		it++;
		ballY = *it;
		it++;
		ballZ = *it;
		it++;
		ballVX = *it;
		it++;
		ballVY = *it;
		it++;
		ballVZ = *it;
		it++;
	}
	void print() {
		cout << "(" << ballX << ":" << ballY << ":" << ballZ << ") ";
		cout << "(" << ballVX << ":" << ballVY << ":" << ballVZ << ")";
	}
};

struct point
{
	int16_t x;
	int16_t y;
	void append(unsigned char* ptr)
	{
		int16_t arr[] = {x, y};
		unsigned char* it = (unsigned char*)&arr[0];
		for (int i = 0; i < opp_size; i++)
		{
			ptr[i] = it[i];
		}
	}

	void desrializeFromPtr(unsigned char* ptr)
	{
		int16_t* it = (int16_t*)&ptr[0];
		x = *it;
		it++;
		y = *it;
	}

	void print() {
		cout << "(" << x << ":" << y << ")";
	}
};

class MultiCastReceive
{
public:
	MultiCastReceive()
	{
	}
	~MultiCastReceive()
	{
	}

	void callback(char* buffer, int size)
	{
		ballPos bp;
		point opps[10];
		point self;
		if(mixed_team_flag_size + ball_size + (opp_size * opp_count) + position_size != size) {
			cout << "strange packet received. Size:" << size << endl;
		}
		unsigned char* it = (unsigned char*)buffer;
		unsigned char flag = *it;
		it++;
		bp.desrializeFromPtr(it);
		it+=ball_size;
		for(int i=0; i<opp_count; i++) {
			opps[i].desrializeFromPtr(it);
			it += opp_size;
		}
		self.desrializeFromPtr(it);


		cout << "Received ("<< size << " Bytes) MID is:" << endl;
		bp.print();
		self.print();
		for(int i=0; i<opp_count; i++) {
			opps[i].print();
		}
		cout << endl;
	}
};


MultiCastChannel<MultiCastReceive>* commandChannel;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void messageCallback(msl_workshop_proxy::TestMessagePtr msg)
{
	unsigned char mixed_team_flag = 123;
	ballPos bp;
	bp.ballX = 1001;
	bp.ballY = 1001;
	bp.ballZ = 1001;
	bp.ballVX = 1001;
	bp.ballVY = 1001;
	bp.ballVZ = 1001;

	point opps[10];

	point self;
	self.x = 1001;
	self.y = 1001;

	unsigned char *arr = new unsigned char[mixed_team_flag_size + ball_size + (opp_size * opp_count) + position_size];
	unsigned char *it = &arr[0];
	it[0] = mixed_team_flag;
	it += 1;
	bp.append(it);
	it += ball_size;
	for (int i = 0; i < opp_count; i++)
	{
		opps[i].x = 1001;
		opps[i].y = 1001;
		opps[i].append(it);
		it += opp_size;
	}
	self.append(it);
	it += opp_size;

	unsigned int packetSize = it - arr;
	if (mixed_team_flag_size + ball_size + (opp_size * opp_count) + position_size != packetSize)
	{
		cout << "strange stuff happend packetsend size is: " << packetSize << endl;
	}

	commandChannel->publish((const char*)arr, packetSize);
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
	ros::Publisher chatter_pub = n.advertise<msl_workshop_proxy::TestMessage>("flooding_test/TestMessage", 10);

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
	commandChannel = new MultiCastChannel<MultiCastReceive>(addr, 50000, &MultiCastReceive::callback, &mcr);

	while (ros::ok())
	{
		ros::spinOnce();
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */
		msl_workshop_proxy::TestMessage msg;

		msg.id = count;
		msg.sender = ownID;

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		//cout << "Sending: " << msg.id << endl;
		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}
