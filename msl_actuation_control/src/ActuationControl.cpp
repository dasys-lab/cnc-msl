#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include "msl_actuator_msgs/BallCatchCmd.h"
#include "msl_actuator_msgs/BallHandleCmd.h"
#include "msl_actuator_msgs/ShovelSelectCmd.h"
#include "msl_actuator_msgs/HaveBallInfo.h"
#include "msl_actuator_msgs/VisionRelocTrigger.h"
#include "msl_actuator_msgs/MotionLight.h"
#include "msl_actuator_msgs/MotionBurst.h"
#include "msl_actuator_msgs/CanMsg.h"
#include "std_msgs/Empty.h"
#include "Can.h"
#include "SystemConfig.h"

#include "ros/ros.h"
#include <ros/transport_hints.h>

#define TIMEDIFFMS(n,o) (((n).tv_sec-(o).tv_sec)*1000+((n).tv_usec-(o).tv_usec)/1000)
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))


#define		CMD_PING			0x01
#define		CMD_SERVO			0x02
#define 	CMD_DRIBBLE			0x03
#define 	CMD_STOP			0x04
#define 	CMD_HAVE_BALL			0x05
#define 	CMD_VISION			0x06
#define 	CMD_BUNDLE			0x07
#define		CMD_MOTION_BURST		0x08
#define		CMD_LIGHT			0x09

ros::Publisher hbPub;
ros::Publisher vtPub;
ros::Publisher brtPub;
ros::Publisher bsPub;
ros::Publisher mbPub;
ros::Publisher canPub;

struct timeval last_bhc_time;
struct timeval last_shovel_time;
struct timeval last_ping_time;
struct timeval cur_time;
uint8_t lastShovel;

struct settings {
int haveLightBarrier;
int robotId;
int maxBallHandlerPWM;
unsigned char passingShovelPos;
unsigned char normalShovelPos;
unsigned char catchBallPos;
unsigned char notCatchBallPos;
int pingInterval;
int shovelInterval;
} settings;

using namespace msl_actuator_msgs;
using namespace supplementary;

class ActuationControl {
	public:

		ActuationControl() {}
		void Start() {
			gettimeofday(&last_bhc_time,NULL);

			SystemConfig* sc = SystemConfig::getInstance();

			Configuration *actconf = (*sc)["Actuation"];
			lastShovel = 50;
			settings.maxBallHandlerPWM = actconf->get<int>("Actuation","BallHandler","MaxPWM",NULL);
			settings.passingShovelPos = actconf->get<int>("Actuation","ShovelSelect","PassingPos",NULL);
			settings.normalShovelPos = actconf->get<int>("Actuation","ShovelSelect","NormalPos",NULL);
			settings.catchBallPos = actconf->get<int>("Actuation","BallCatching","CatchingPos",NULL);
			settings.notCatchBallPos = actconf->get<int>("Actuation","BallCatching","UpPos",NULL);
			settings.pingInterval = actconf->get<int>("Actuation","PingInterval",NULL);
			settings.shovelInterval = actconf->get<int>("Actuation","ShovelSelectRepeatInterval",NULL);
			settings.haveLightBarrier = actconf->get<int>("Actuation","HaveLightBarrier",NULL);
			settings.robotId = sc->getOwnRobotID();

			ros::NodeHandle node;

			actuatorCanSub = node.subscribe<CanMsg, ActuationControl >("/usb_can_proxy/BallHandler", 30,&ActuationControl::handleCanActuator,this);
			
			bhcSub = node.subscribe< BallHandleCmd, ActuationControl >("BallHandleControl", 30,&ActuationControl::handleBallHandleControlMessage,this);
			bccSub = node.subscribe<BallCatchCmd,ActuationControl >("BallCatchControl", 30,&ActuationControl::handleBallCatchControlMessage,this);
			//shcSub = node.subscribe<ShovelSelectCmd,ActuationControl >("ShovelSelectControl", 30,&ActuationControl::handleShovelSelectControlMessage,this,ros::TransportHints().udp());
			shcSub = node.subscribe<ShovelSelectCmd,ActuationControl >("ShovelSelectControl", 30,&ActuationControl::handleShovelSelectControlMessage,this);
			motionlightSub = node.subscribe<MotionLight,ActuationControl>("CNActuator/MotionLight", 30,&ActuationControl::handleMotionLightControlMessage,this);
			std::cout << "Add callback done" << std::endl;
			hbPub = node.advertise<HaveBallInfo>("HaveBallInfo", 1);
			vtPub = node.advertise<VisionRelocTrigger>("CNActuator/VisionRelocTrigger", 10);
			brtPub = node.advertise<std_msgs::Empty>("CNActuator/BundleRestartTrigger", 10);
			bsPub = node.advertise<VisionRelocTrigger>("CNActuator/BundleStatus", 10);
			mbPub = node.advertise<MotionBurst>("CNActuator/MotionBurst", 10);
			canPub = node.advertise<CanMsg>("usb_can_proxy/CanSub", 30);
			//change 
			//canPub = node.advertise<CanMsg>("CNUsbCanProxy/CanSub", 30);

			spinner = new ros::AsyncSpinner(1);
			spinner->start();

		}

	protected:
		ros::AsyncSpinner *spinner;
		ros::Subscriber bhcSub;
		ros::Subscriber bccSub;
		ros::Subscriber shcSub;
		ros::Subscriber motionlightSub;
		ros::Subscriber actuatorCanSub;
				
		void handleCanActuator(const CanMsg message)
		{
			int command = message.data[0];
			//printf("actuator: get val from proxy %d\n",command);
		//printf("lb : %d\n",settings.haveLightBarrier);	
			//if( command == CMD_HAVE_BALL && settings.haveLightBarrier > 0)
			if( command == CMD_HAVE_BALL )
			{
				HaveBallInfo info;
				if( (int)message.data[1] )
					info.haveBall = true;
				else
					info.haveBall = false;
				
				hbPub.publish(info);
			}
			else if( command == CMD_VISION )
			{
				VisionRelocTrigger vt;
				vt.receiverID = settings.robotId;
				vt.usePose = true;
				
				vtPub.publish(vt);
			}
			else if( command == CMD_BUNDLE )
			{
				//TODO add bundle stuff
				// send void msg to Care
				std_msgs::Empty brtMsg;
				brtPub.publish(brtMsg);
			}
			else if( command == CMD_MOTION_BURST )
			{
				MotionBurst mb;
				mb.x = (message.data[1] << 8) + message.data[2] - 32767;
				mb.y = (message.data[3] << 8 ) + message.data[4] - 32767;
				mb.qos = (message.data[5] << 8 ) + message.data[6] - 32767;
				//printf("%d %d %d",mb.x, mb.y, mb.qos);
				mbPub.publish(mb);
			}
		}
		
		void handleBallHandleControlMessage(const BallHandleCmd::ConstPtr& message){
		//	std::cout << "ActControl " << message->getHostId()<<"\n";
		//	std::cout << "BH RCVD" << std::endl;
			char lm = message->leftMotor;
			char rm = message->rightMotor;
			if (lm > settings.maxBallHandlerPWM) {
				lm = settings.maxBallHandlerPWM;
			} else if (lm < - settings.maxBallHandlerPWM) {
				lm = - settings.maxBallHandlerPWM;
			}
			if (rm > settings.maxBallHandlerPWM) {
				rm = settings.maxBallHandlerPWM;
			} else if (rm < - settings.maxBallHandlerPWM) {
				rm = - settings.maxBallHandlerPWM;
			}

			CanMsg cm;
			cm.id = BallHandler;
			cm.data.push_back(CMD_DRIBBLE);
			cm.data.push_back(lm);
			cm.data.push_back(rm);

			canPub.publish(cm);
			gettimeofday(&last_bhc_time,NULL);
			return;
		}

		void handleBallCatchControlMessage(const BallCatchCmd::ConstPtr& message){
		//	std::cout << "ActControl " << message->getHostId()<<"\n";
		//	std::cout << "BC RCVD" << std::endl;
			CanMsg cm;
			cm.id = BallHandler;
			cm.data.push_back(CMD_STOP);
			if (message->down) {
				cm.data.push_back(settings.catchBallPos);
			} else cm.data.push_back(settings.notCatchBallPos);
			
			canPub.publish(cm);
			return;
		}
		void handleMotionLightControlMessage(const MotionLight::ConstPtr& message){
			printf("motion light is : %d\n", message->enable);
			CanMsg cm;
			cm.id = BallHandler;
			cm.data.push_back(CMD_LIGHT);
			//printf("is : %d\n",message->enable);	
			if( message->enable )
			{
				cm.data.push_back(1);
			}
			else
			{
				cm.data.push_back(0);
			}
			//printf("all: \n");
			//for(unsigned int i=0; i<cm.data.size(); i++)
			//{
			//	printf("data are : %d\n",cm.data[i]);
			//}
			canPub.publish(cm);
			return;
		}
		void handleShovelSelectControlMessage(const ShovelSelectCmd::ConstPtr& message){
		//	std::cout << "ActControl " << message->getHostId()<<"\n";
		//	std::cout << "SH RCVD" << std::endl;
			CanMsg cm;
			cm.id = BallHandler;
			cm.data.push_back(CMD_SERVO);
			if (message->passing) {
				cm.data.push_back(settings.passingShovelPos);
				lastShovel = settings.passingShovelPos;
			} 
			else 
			{
				cm.data.push_back(settings.normalShovelPos);
				lastShovel = settings.normalShovelPos;
			}

			canPub.publish(cm);
			
			gettimeofday(&last_shovel_time,NULL);	
			return;
		}
};

ActuationControl *ac;
pthread_t reading;
void int_handler(int sig) {
	pthread_join( reading, NULL);
	exit(2);
}

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "ActuationControl");
	
	ac = new ActuationControl();
	ac->Start();
	int pingdelta;
	int shoveldelta;
	int sleeptime;
	gettimeofday(&last_ping_time,NULL);
	last_shovel_time = last_ping_time;
		
	while(ros::ok()) {
		gettimeofday(&cur_time,NULL);
		pingdelta = TIMEDIFFMS(cur_time,last_ping_time);
		shoveldelta = TIMEDIFFMS(cur_time,last_shovel_time);
		if (pingdelta > settings.pingInterval) {
			CanMsg msg;
			msg.id = BallHandler;
// 			msg.data.push_back(0);
// 			msg.data.push_back(CanPriNorm);
// 			msg.data.push_back(Eth2Can);
// 			msg.data.push_back(BallHandler);
// 			msg.data.push_back(1);
			msg.data.push_back(CMD_PING);
			
			canPub.publish(msg);
			
			last_ping_time = cur_time;
		}		
		if (TIMEDIFFMS(cur_time,last_bhc_time) > 1000) {
//std::cout << "Didn't get a msg for 1 sec! Acutual diff is: '" << TIMEDIFFMS(cur_time,last_bhc_time) << "'" << std::endl;
			
			CanMsg msg;
			msg.id = BallHandler;
// 			msg.data.push_back(0);
// 			msg.data.push_back(CanPriNorm);
// 			msg.data.push_back(Eth2Can);
// 			msg.data.push_back(BallHandler);
// 			msg.data.push_back(3);
			msg.data.push_back(CMD_DRIBBLE);
			msg.data.push_back(0);
			msg.data.push_back(0);

			canPub.publish(msg);
			
			last_bhc_time = cur_time;
		}
		if (shoveldelta > settings.shovelInterval) {
			
			CanMsg msg;
			msg.id = BallHandler;
// 			msg.data.push_back(0);
// 			msg.data.push_back(CanPriNorm);
// 			msg.data.push_back(Eth2Can);
// 			msg.data.push_back(BallHandler);
// 			msg.data.push_back(2);
			msg.data.push_back(CMD_SERVO);
			msg.data.push_back(lastShovel);

			canPub.publish(msg);
		
			last_shovel_time = cur_time;
		}
		
		sleeptime = MAX(1, MIN(settings.shovelInterval - TIMEDIFFMS(cur_time,last_shovel_time), settings.pingInterval - TIMEDIFFMS(cur_time,last_ping_time) ) ) *1000 ;
//std::cout << pingdelta << " " << shoveldelta<<" "<<sleeptime<<std::endl;
		usleep(sleeptime);
		//usleep(settings.pingInterval*1000);
	}

}
