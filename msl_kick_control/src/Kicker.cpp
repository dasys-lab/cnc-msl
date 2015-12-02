#include "Kicker.h"

using namespace std;
using namespace supplementary;
using namespace msl_actuator_msgs;

string msldriver::Kicker::msg2string(vector<uint8_t> data)
{
	string s(data.begin(), data.end());
	return s;
}

void msldriver::Kicker::onKickerMsg(const CanMsg message)
{
	if (message.data.size() == 0)
		return;

	int sw = (int)message.data[0];

	//declare vars for sw
	short boosterState;
	double supplyVoltage;
	short capacitorsVoltage;
	bool supplyOn;
	bool boostPWMOn;
	bool releaseOn;
	bool error;
	int rotate;
	string msg;
	string warn;
	string debugmessage;
	string reset;
	string err;
	string version;

	switch (sw)
	{
		case Pong:
			lastBeat = ros::Time::now();
			break;
		case State:
			if (message.data.size() != 5 && message.data.size() != 6)
			{
				ROS_ERROR("ERROR: wrong state-packet format");
			}

			boosterState = (short)message.data[1];
			supplyVoltage = ((message.data[3] << 8) + message.data[2]) * 0.0394629;
			capacitorsVoltage = 0;
			if (message.data.size() == 6)
			{
				capacitorsVoltage = (short)(message.data[4] + ((short)(message.data[5] << 8)));
			}
			else
			{
				capacitorsVoltage = (short)message.data[4];
			}

			supplyOn = ((boosterState & (1 << 7)) > 0);
			boostPWMOn = ((boosterState & (1 << 6)) > 0);
			releaseOn = ((boosterState & (1 << 5)) > 0);
			error = ((boosterState & (1 << 4)) > 0);

			rotate = (int)(boosterState & 0x03);

			lastCapacitorsVoltage = (double)capacitorsVoltage;
			lastSupplyVoltage = supplyVoltage;

			lastKickStatUpdate = ros::Time::now();
			break;
		case Error:
			err = msg2string(message.data);
			err.erase(0);
			rekickerror += err;
			if (rekickerror[rekickerror.size() - 1] == '\n')
			{
				rekickerror.erase(rekickerror.size() - 1);
				ROS_ERROR("ReKick Error : %s", rekickerror.c_str());
				rekickerror = "";
			}
			break;
		case Warning:
			warn = msg2string(message.data);
			warn.erase(0);
			rekickwarn += warn;
			if (rekickwarn[rekickwarn.size() - 1] == '\n')
			{
				rekickwarn.erase(rekickwarn.size() - 1);
				ROS_INFO("Warning: %s", rekickwarn.c_str());
				rekickwarn = "";
			}
			break;
		case Msg:
			msg = msg2string(message.data);
			msg.erase(0);
			rekickmessage += msg;
			if (rekickmessage[rekickmessage.size() - 1] == '\n')
			{
				rekickmessage.erase(rekickmessage.size() - 1);
				debugmessage = rekickmessage;
				rekickmessage = "";
				ROS_DEBUG("ReKick Message: %s", debugmessage.c_str());
				if (strncmp(debugmessage.c_str(), "Kicktime:", 9) == 0)
				{

					ROS_DEBUG("Driver time: %F", (ros::Time::now().toSec() - lastKickRequest.toSec()));
				}
				lastKickRequest = ros::Time::now();
			}
			break;
		case Reset:
			switch ((int)message.data[1])
			{
				case 0:
					reset = "Power on reset";
					break;
				case 1:
					reset = "Extern reset";
					break;
				case 2:
					reset = "Brown out reset";
					break;
				case 3:
					reset = "Watchdog reset";
					break;
				case 4:
					reset = "JTAG reset";
					break;
				default:
					reset = "undef";
					break;
			}
			ROS_ERROR("ReKick Reset occured. Reason: %s. Reinitialize now.", reset.c_str());
			break;
		default:
			ROS_ERROR("Received unknown packet: 0x{%x}", message.data[0]);
	}
}

void msldriver::Kicker::onKickCmd(const msl_actuator_msgs::KickControl ks)
{
	ros::Time now = ros::Time::now();

	if ((lastKickRequest.toSec() + 0.1 > now.toSec()))
	{
		return;
	}

	lastKickRequest = now;
	lastPower = ks.power;

	ROS_DEBUG("Got KickerMessage: kicker: %d power: %d", ks.kicker, ks.power);

	CanMsg cm;
	cm.id = ReKick;

	for (int i = 0; i < 5; i++)
	{
		cm.data.clear();
		if (settings.highResolution)
		{
			int power;
			if (ks.power > 0 && ks.power <= 100)
			{
				power = (ks.power * 100) / 3;
			}
			else if (ks.power > 0 && ks.power < 3500)
			{
				power = ks.power;
			}
			else
			{
				ROS_ERROR("Kick Power too high: %d", ks.power);
				return;
			}

			if (ks.forceVoltage > 0)
			{
				cm.data.push_back(Kick);
				cm.data.push_back((power & 0xFF));
				cm.data.push_back(((power >> 8) & 0xFF));
				cm.data.push_back(ks.forceVoltage);
			}
			else
			{
				cm.data.push_back(Kick);
				cm.data.push_back((power & 0xFF));
				cm.data.push_back(((power >> 8) & 0xFF));
			}
			rekickPub.publish(cm);
		}
		else if (ks.power > 0 && ks.power <= 100)
		{
			// 30 ms =~ 100 %
			int ms = ks.power / 3;
			if (ks.forceVoltage > 0)
			{
				cm.data.push_back(Kick);
				cm.data.push_back(ms);
				cm.data.push_back(ks.forceVoltage);
			}
			else
			{
				cm.data.push_back(Kick);
				cm.data.push_back(ms);
			}
			rekickPub.publish(cm);
		}
		ros::Duration(0.001).sleep();
	}
}

void msldriver::Kicker::setMaxVoltage()
{
	if (settings.maxVoltage > 0)
	{
		CanMsg cm;
		cm.id = ReKick;
		if (settings.maxVoltage > 255)
		{
			cm.data.push_back(SetMaxVoltage);
			cm.data.push_back((settings.maxVoltage & 0xFF));
			cm.data.push_back((settings.maxVoltage >> 8));
		}
		else
		{
			cm.data.push_back(SetMaxVoltage);
			cm.data.push_back(settings.maxVoltage);
		}
		rekickPub.publish(cm);
	}
}

msldriver::Kicker::Kicker() : lastCapacitorsVoltage(0),lastPower (0), lastSupplyVoltage(0)
{
	//init vars via system_config
	SystemConfig* sc = SystemConfig::getInstance();

	Configuration *kickerConf = (*sc)["Kicker"];

	settings.alivePeriod = kickerConf->get<int>("Kicker", "Alive Period", NULL);
	settings.highResolution = kickerConf->get<bool>("Kicker", "HighResolution", NULL);
	settings.maxVoltage = kickerConf->get<int>("Kicker", "ReKick", "Maximum Charging Voltage", NULL);
	settings.robotId = sc->getOwnRobotID();

	ROS_INFO("Kicker Params:");
	ROS_INFO("Alive period : %i", settings.alivePeriod);
	ROS_INFO("HighResolution : %i", settings.highResolution);
	ROS_INFO("Max voltage : %i", settings.maxVoltage);

	// init publisher and subscriber
	this->rosNode = new ros::NodeHandle();
	this->sub = rosNode->subscribe("usb_can_proxy/Rekick", 30, &msldriver::Kicker::onKickerMsg, (msldriver::Kicker*)this);
	this->kickSub = rosNode->subscribe("KickControl", 30, &msldriver::Kicker::onKickCmd, (msldriver::Kicker*)this);
	//Change this to other subscribe
	//this->rekickPub = rosNode->advertise<CanMsg>("CNUsbCanProxy/CanSub", 30);
	this->rekickPub = rosNode->advertise<CanMsg>("usb_can_proxy/CanSub", 30);
	this->kickerTime = rosNode->advertise<msl_actuator_msgs::KickTime>("KickTime", 30);
	this->kickerStat = rosNode->advertise<msl_actuator_msgs::KickerStatInfo>("KickerStatInfo", 30);
}

msldriver::Kicker::~Kicker()
{
	delete this->rosNode;
}

void msldriver::Kicker::setLastBeat()
{
	this->lastBeat = ros::Time::now();
}

void msldriver::Kicker::sendHeartBeat(ros::Time now)
{
	if ((now.toSec() - this->lastBeat.toSec()) > (this->settings.alivePeriod / 1000))
	{
		// check state
		CanMsg cm2;
		cm2.id = ReKick;
		cm2.data.push_back(GetState);
		this->rekickPub.publish(cm2);

		ros::Duration(0.001).sleep();

		//send heartbeat
		CanMsg cm;
		cm.id = ReKick;
		cm.data.push_back(Ping);
		this->rekickPub.publish(cm);
	}
}

void msldriver::Kicker::sendKickTime(ros::Time now)
{
	if (now.toSec() - this->lastKickTimeInfo.toSec() > 0.75)
	{
		this->lastKickTimeInfo = now;
		msl_actuator_msgs::KickTime kt;
		kt.time = lastKickRequest;
		this->kickerTime.publish(kt);
	}
}

void msldriver::Kicker::checkStatus(ros::Time now)
{
	if (now.toSec() - this->lastKickStatUpdate.toSec() > 4.00)
	{
		this->lastSupplyVoltage = 0.0;
		this->lastCapacitorsVoltage = 0.0;
	}
}

void msldriver::Kicker::sendStatus(ros::Time now)
{
	if (now.toSec() - this->lastStatInfo.toSec() > 1.00)
	{
		this->lastStatInfo = now;
		msl_actuator_msgs::KickerStatInfo ksi;
		ksi.senderID = settings.robotId;
		ksi.supplyVoltage = lastSupplyVoltage;
		ksi.capVoltage = lastCapacitorsVoltage;
		this->kickerStat.publish(ksi);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "msl_kick_control");
	msldriver::Kicker* kicker = new msldriver::Kicker();

	kicker->setMaxVoltage();
	kicker->setLastBeat();

	ros::Rate loop_rate(60);
	while (ros::ok())
	{
		ros::Time now = ros::Time::now();

		//send heartbeat
		kicker->sendHeartBeat(now);

		//send kicktime
		kicker->sendKickTime(now);

		//check status
		kicker->checkStatus(now);

		//send status
		kicker->sendStatus(now);

		ros::spinOnce();

		loop_rate.sleep();
	}

	delete kicker;
}
