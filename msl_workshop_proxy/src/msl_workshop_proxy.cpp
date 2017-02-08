/*
 * flooding_test_node.cpp
 *
 *  Created on: 27.10.2014
 *      Author: endy
 */

#include "CaceMultiCastChannel.h"
#include "DataStructures.h"
#include "geometry_msgs/Point32.h"
#include "msl_sensor_msgs/WorldModelData.h"
#include "ros/ros.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/String.h"
#include <iostream>

#include <map>
#include <sstream>

using namespace std;
using namespace multicast;
using namespace sensor_msgs;
using namespace geometry_msgs;

ros::Publisher ballPub;
ros::Publisher selfPub;
ros::Publisher obstaclesPub;

int sendCounter = 0;

int ownID = 4;
unsigned long timeout = 1000000000;
map<int, ballPos> ballPositions;
map<int, point> robotPositions;
vector<point> allOpps;
map<int, unsigned long> lastUpdateTime;

point allo2Ego(point &p, msl_msgs::PositionInfo &ownPos)
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

point ego2Allo(point p, msl_msgs::PositionInfo &ownPos)
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
    void callback(char *buffer, int size)
    {
        ballPos bp;
        point opps[10];
        point self;
        if (mixed_team_flag_size + ball_size + (opp_size * opp_count) + position_size != size)
        {
            /*cout << "strange packet received. Size:" << size << " but should be: "
             << mixed_team_flag_size + ball_size + (opp_size * opp_count) + position_size << " from robot: ";
             if (size > 2)
             cout << buffer[1] << endl;*/
            return;
        }
        unsigned char *it = (unsigned char *)buffer;
        unsigned char flag = *it;
        if (flag != 123)
        {
            cout << "received strange mixed team flag" << endl;
            return;
        }
        it++;
        int robotID = *it;
        cout << (int)robotID << flush;
        if (robotID > 6)
        {
            cout << "received strange robotID" << endl;
            return;
        }
        it++;
        bp.desrializeFromPtr(it);
        it += ball_size;
        for (int i = 0; i < opp_count; i++)
        {
            opps[i].desrializeFromPtr(it);
            it += opp_size;
        }
        self.desrializeFromPtr(it);

        auto now = std::chrono::high_resolution_clock::now();
        auto pointinTime = now.time_since_epoch();
        unsigned long curTime = std::chrono::duration_cast<std::chrono::nanoseconds>(pointinTime).count();

        ballPositions[robotID] = bp;
        robotPositions[robotID] = self;
        lastUpdateTime[robotID] = curTime;

        for (auto &copp : opps)
        {
            if (robotID == ownID || ((fabs(copp.x - self.x * 1000.0) < 1200 && fabs(copp.y - self.y * 1000.0) < 1200)))
            {
                allOpps.push_back(copp);
            }
        }

        if (robotID != ownID)
        {
            return;
        }
        // sending visualisation data
        PointCloud ballCloud, ownPosition, obstacles;
        ballCloud.header.frame_id = "/map";
        ownPosition.header = ballCloud.header;
        obstacles.header = ballCloud.header;

        {
            if (bp.ballX != -32768 || bp.ballY != -32768)
            {
                Point32 p;
                p.x = bp.ballX / 1000.0;
                p.y = bp.ballY / 1000.0;
                p.z = 120.0 / 1000.0;
                ChannelFloat32 chan;
                chan.name = "ball";
                ballCloud.points.push_back(p);
                ballCloud.channels.push_back(chan);
                ballPub.publish(ballCloud);
            }
            else
            {
                ballPos bestBP;
                for (auto item : ballPositions)
                {
                    if ((curTime - lastUpdateTime[item.first]) < 3 * timeout)
                    {
                        if (item.second.ballX != -32768 && item.second.ballY != -32768)
                        {
                            if (item.second.confidence >= bestBP.confidence)
                            {
                                bestBP = item.second;
                            }
                        }
                    }
                }
            }
        }

        {
            for (auto item : robotPositions)
            {
                Point32 p;
                p.x = item.second.x / 1000.0;
                p.y = item.second.y / 1000.0;
                p.z = 250.0 / 1000.0;
                ChannelFloat32 chan;
                chan.name = "self";
                chan.values.push_back(item.first);
                if ((curTime - lastUpdateTime[item.first]) < timeout)
                {
                    ownPosition.points.push_back(p);
                    ownPosition.channels.push_back(chan);
                }
            }
            selfPub.publish(ownPosition);
        }

        {
            for (point &copp : allOpps)
            {
                if (copp.x == -32768 || copp.y == -32768)
                {
                    continue;
                }
                Point32 pa;
                pa.x = copp.x / 1000.0;
                pa.y = copp.y / 1000.0;
                pa.z = 250.0 / 1000.0;
                ChannelFloat32 chan;
                chan.name = "opps";
                bool isOpp = true;
                for (auto item : robotPositions)
                {
                    if ((curTime - lastUpdateTime[item.first]) < timeout)
                    {
                        if (fabs(item.second.x - pa.x * 1000.0) < 350 && fabs(item.second.y - pa.y * 1000.0) < 350)
                        {
                            isOpp = false;
                        }
                    }
                }
                for (auto &alreadyadded : obstacles.points)
                {
                    if (fabs(alreadyadded.x * 1000.0 - pa.x * 1000.0) < 350 && fabs(alreadyadded.y * 1000.0 - pa.y * 1000.0) < 350)
                    {
                        isOpp = false;
                    }
                }
                if (isOpp)
                {
                    obstacles.points.push_back(pa);
                    obstacles.channels.push_back(chan);
                }
            }
            obstaclesPub.publish(obstacles);
            allOpps.clear();
        }
    }
};

MultiCastChannel<MultiCastReceive> *commandChannel;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void messageCallback(msl_sensor_msgs::WorldModelDataPtr msg)
{
    if (++sendCounter % 3 != 0)
    {
        return;
    }
    cout << "." << flush;
    // setup data to serialize
    unsigned char robotID = ownID;
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
    bp.ballVY = (cos(msg->odometry.position.angle) * msg->ball.velocity.vx - sin(msg->odometry.position.angle) * msg->ball.velocity.vy);
    bp.ballVX = -(sin(msg->odometry.position.angle) * msg->ball.velocity.vx + cos(msg->odometry.position.angle) * msg->ball.velocity.vy);
    bp.ballVZ = msg->ball.velocity.vz;
    bp.confidence = (uint8_t)(msg->ball.confidence * 255.0);

    point opps[10];
    point self;
    self.y = -msg->odometry.position.x;
    self.x = msg->odometry.position.y;
    self.confidence = (uint8_t)(msg->odometry.position.certainty * 255.0);

    // serialize
    unsigned char *arr = new unsigned char[mixed_team_flag_size + ball_size + (opp_size * opp_count) + position_size];
    unsigned char *it = &arr[0];
    it[0] = mixed_team_flag;
    it += 1;
    it[0] = robotID;
    it += 1;
    bp.append(it);
    it += ball_size;
    for (int i = 0; i < opp_count; i++)
    {
        if (i < msg->obstacles.size())
        {
            opps[i].y = msg->obstacles[i].y;
            opps[i].x = msg->obstacles[i].x;
            opps[i].confidence = 128;
            opps[i] = ego2Allo(opps[i], msg->odometry.position);
            swap(opps[i].y, opps[i].x);
            opps[i].y = -opps[i].y;
        }
        else
        {
            opps[i].x = -32768;
            opps[i].y = -32768;
            opps[i].confidence = 0;
        }
        opps[i].append(it);
        it += opp_size;
    }
    self.append(it);
    it += opp_size;

    // check
    unsigned int packetSize = it - arr;
    if (mixed_team_flag_size + ball_size + (opp_size * opp_count) + position_size != packetSize)
    {
        cout << "strange stuff happend packetsend size is: " << packetSize << endl;
    }

    // send via multicast
    commandChannel->publish((const char *)arr, packetSize);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "msl_workshop_proxy");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<msl_sensor_msgs::WorldModelData>("/WorldModel/WorldModelData", 1);

    ballPub = n.advertise<PointCloud>("/ball", 1);
    selfPub = n.advertise<PointCloud>("/self", 1);
    obstaclesPub = n.advertise<PointCloud>("/obstacles", 1);

    ros::Rate loop_rate(30);
    ros::Subscriber sub = n.subscribe("/WorldModel/WorldModelData", 1, messageCallback);

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

        // ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
