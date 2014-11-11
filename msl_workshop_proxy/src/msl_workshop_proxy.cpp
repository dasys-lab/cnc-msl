/*
 * flooding_test_node.cpp
 *
 *  Created on: 27.10.2014
 *      Author: endy
 */

#include <iostream>
#include "DataStructures.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "msl_sensor_msgs/WorldModelData.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "CaceMultiCastChannel.h"

#include <sstream>

using namespace std;
using namespace multicast;
using namespace sensor_msgs;
using namespace geometry_msgs;

ros::Publisher ballPub;
ros::Publisher selfPub;
ros::Publisher obstaclesPub;

int sendCounter = 0;

point allo2Ego(point& p, msl_msgs::PositionInfo& ownPos)
{
	point ego;

	double x = p.x - ownPos.x;
	double y = p.y - ownPos.y;

	double angle = atan2(y, x) - ownPos.angle;
	double dist = sqrt(x * x + y * y);

	ego.x = cos(angle) * dist;
	ego.y = sin(angle) * dist;

	return ego;
}

point ego2Allo(point p, msl_msgs::PositionInfo& ownPos)
{

	point allo;
	allo.x = ownPos.x;
	allo.y = ownPos.y;

	allo.x += cos(ownPos.angle) * p.x - sin(ownPos.angle) * p.y;
	allo.y += sin(ownPos.angle) * p.x + cos(ownPos.angle) * p.y;

	return allo;
}

class MultiCastReceive
{
public:
	void callback(char* buffer, int size)
	{
		cout << "X" << flush;
		ballPos bp;
		point opps[10];
		point self;
		if (mixed_team_flag_size + ball_size + (opp_size * opp_count) + position_size != size)
		{
			cout << "strange packet received. Size:" << size << " but should be: " << mixed_team_flag_size + ball_size + (opp_size * opp_count) + position_size << endl;
		}
		unsigned char* it = (unsigned char*)buffer;
		unsigned char flag = *it;
		it++;
		int robotID = *it;
		it++;
		bp.desrializeFromPtr(it);
		it += ball_size;
		for (int i = 0; i < opp_count; i++)
		{
			opps[i].desrializeFromPtr(it);
			it += opp_size;
		}
		self.desrializeFromPtr(it);

		/*cout << "Received (" << size << " Bytes) MID is:" << endl;
		bp.print();
		cout << "-";
		self.print();
		cout << "-";
		for (int i = 0; i < opp_count; i++)
		{
			opps[i].print();
		}
		cout << endl;*/
	}
};

MultiCastChannel<MultiCastReceive>* commandChannel;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void messageCallback(msl_sensor_msgs::WorldModelDataPtr msg)
{
	if(++sendCounter % 10 != 0) {
		return;
	}
	cout << "." << flush;
	//setup data to serialize
	unsigned char robotID = 5;
	unsigned char mixed_team_flag = 123;
	ballPos bp;
	point b;
	b.x = msg->ball.point.x;
	b.y = msg->ball.point.y;
	b = ego2Allo(b, msg->odometry.position);
	swap(b.x, b.y);
	b.y = -b.y;

	bp.ballX = b.x;
	bp.ballY = b.y;
	bp.ballZ = msg->ball.point.z;
	bp.ballVY = (cos(msg->odometry.position.angle) * msg->ball.velocity.vx
			- sin(msg->odometry.position.angle) * msg->ball.velocity.vy);
	bp.ballVX = -(sin(msg->odometry.position.angle) * msg->ball.velocity.vx
			+ cos(msg->odometry.position.angle) * msg->ball.velocity.vy);
	bp.ballVZ = msg->ball.velocity.vz;
	bp.confidence = (uint8_t)(msg->ball.confidence*255.0);

	point opps[10];
	point self;
	self.y = msg->odometry.position.x;
	self.x = -msg->odometry.position.y;
	self.confidence = (uint8_t)(msg->odometry.position.certainty*255.0);

	//serialize
	unsigned char *arr = new unsigned char[mixed_team_flag_size + ball_size + (opp_size * opp_count) + position_size];
	unsigned char *it = &arr[0];
	it[0] = mixed_team_flag;
	it += 1;
	it[0] = robotID;
	bp.append(it);
	it += 1;
	it += ball_size;
	for (int i = 0; i < opp_count; i++)
	{
		if (i < msg->obstacles.size())
		{
			opps[i].y = msg->obstacles[i].x;
			opps[i].x = -msg->obstacles[i].y;
			bp.confidence = (uint8_t)(msg->obstacles[i].diameter*255.0);
			opps[i] = ego2Allo(opps[i], msg->odometry.position);
		}
		else
		{
			opps[i].x = 0;
			opps[i].y = 0;
			bp.confidence = 0;
		}
		opps[i].append(it);
		it += opp_size;
	}
	self.append(it);
	it += opp_size;

	//check
	unsigned int packetSize = it - arr;
	if (mixed_team_flag_size + ball_size + (opp_size * opp_count) + position_size != packetSize)
	{
		cout << "strange stuff happend packetsend size is: " << packetSize << endl;
	}

	//send via multicast
	commandChannel->publish((const char*)arr, packetSize);

	//sending visualisation data
	PointCloud ballCloud, ownPosition, obstacles;
	ballCloud.header.frame_id = "/map";
	ownPosition.header = ballCloud.header;
	obstacles.header = ballCloud.header;

	{
		Point32 p;
		p.x = bp.ballX/1000.0;
		p.y = bp.ballY/1000.0;
		p.z = bp.ballZ/1000.0;
		ChannelFloat32 chan;
		chan.name = "ball";
		ballCloud.points.push_back(p);
		ballCloud.channels.push_back(chan);
		ballPub.publish(ballCloud);
	}

	{
		Point32 p;
		p.x = self.x/1000.0;
		p.y = self.y/1000.0;
		p.z = 0/1000.0;
		ChannelFloat32 chan;
		chan.name = "self";
		ownPosition.points.push_back(p);
		ownPosition.channels.push_back(chan);
		selfPub.publish(ownPosition);
	}

	{
		for (int i = 0; i < msg->obstacles.size(); i++)
		{
			Point32 p;
			p.x = opps[i].x/1000.0;
			p.y = opps[i].y/1000.0;
			p.z = 0/1000.0;
			ChannelFloat32 chan;
			chan.name = "opps";
			obstacles.points.push_back(p);
			obstacles.channels.push_back(chan);
		}
		obstaclesPub.publish(obstacles);
	}

}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "msl_workshop_proxy");

	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<msl_sensor_msgs::WorldModelData>("WorldModel/WorldModelData", 1);

	ballPub = n.advertise<PointCloud>("/ball", 1);
	selfPub = n.advertise<PointCloud>("/self", 1);
	obstaclesPub = n.advertise<PointCloud>("/obstacles", 1);

	ros::Rate loop_rate(30);
	ros::Subscriber sub = n.subscribe("WorldModel/WorldModelData", 1, messageCallback);

	MultiCastReceive mcr;
	string addr = "224.16.32.75";
	unsigned short port = 2005;
	commandChannel = new MultiCastChannel<MultiCastReceive>(addr, port, &MultiCastReceive::callback, &mcr);

	while (ros::ok())
	{
		ros::spinOnce();
		/*msl_sensor_msgs::WorldModelData msg;
		msg.odometry.position.x = 1;
		msg.odometry.position.y = 1;
		msg.odometry.position.angle = 3.14159265 / 4.0;

		msg.ball.point.x = 1000;
		msg.ball.point.y = 0;
		msg.ball.velocity.vx = 0;
		msg.ball.velocity.vy = 1000;
		chatter_pub.publish(msg);*/

		//ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
