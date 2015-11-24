/*
 * GameData.h
 *
 *  Created on: May 28, 2015
 *      Author: Stephan Opfer
 */

#ifndef CNC_MSL_RQT_MSL_REFBOX_SRC_RQT_MSL_REFBOX_GAMEDATA_H_
#define CNC_MSL_RQT_MSL_REFBOX_SRC_RQT_MSL_REFBOX_GAMEDATA_H_

#include "ros/ros.h"
#include <map>
#include <QFile>
#include <QtGui>
#include <rqt_msl_refbox/RefBox.h>
#include <QTcpSocket>
#include <QUdpSocket>
#include "rqt_msl_refbox/tinyxml2.h"
#include "rqt_msl_refbox/XMLProtocolParser.h"
#include "msl_msgs/RefBoxCommand.h"
#include "msl_sensor_msgs/SharedWorldInfo.h"
#include "alica_ros_proxy/AlicaEngineInfo.h"
#include <mutex>
#include <QTimer>


using namespace std;

namespace rqt_msl_refbox
{
	class RefBox;
	class XMLProtocolParser;
	class GameData : public QObject
	{
		Q_OBJECT
	public:
		void onSharedWorldmodelInfo(msl_sensor_msgs::SharedWorldInfoPtr msg);
		void onAlicaEngineInfo(alica_ros_proxy::AlicaEngineInfoConstPtr aei);
		void processCharacterBasedProtocol(const char * data);
		void sendCyanCornerKick();
		void sendCyanThrownin();
		void sendStart();
		void sendStop();
		void sendHalt();
		void sendDroppedBall();
		void sendParking();
		void sendMagentaKickOff();
		void sendMagentaFreeKick();
		void sendMagentaGoalKick();
		void sendMagentaThrownin();
		void sendMagentaCornerKick();
		void sendMagentaPenalty();
		void sendCyanKickOff();
		void sendCyanFreeKick();
		void sendCyanGoalKick();
		void sendCyanPenalty();

	public:
		GameData(RefBox* refBox);
		virtual ~GameData();
		RefBox* refBox;
		bool localToggled;
		bool udpToggled;
		bool tcpToggled;
		bool xmlToggled;
		bool charToggled;

	public Q_SLOTS:
		void PlayOnPressed(void);
		void StopPressed(void);
		void HaltPressed(void);
		void DroppedBallPressed(void);
		void ParkingPressed(void);
		void JoystickPressed(void);

		/* our */
		void OurKickOffPressed(void);
		void OurFreeKickPressed(void);
		void OurGoalKickPressed(void);
		void OurThrowinPressed(void);
		void OurCornerKickPressed(void);
		void OurPenaltyPressed(void);

		/* their */
		void TheirKickOffPressed(void);
		void TheirFreeKickPressed(void);
		void TheirGoalKickPressed(void);
		void TheirThrowinPressed(void);
		void TheirCornerKickPressed(void);
		void TheirPenaltyPressed(void);

		void onLocalToggled(bool checked);
		void onUdpToggled(bool checked);
		void onTcpToggled(bool checked);
		void onXmlToggled(bool checked);
		void onCharToggled(bool checked);

		void onConnectPressed(void);
		void receiveRefMsgTcp(void);
		void receiveRefMsgUdp(void);
		void onDisconnectPressed(void);

		void sendRefBoxCmd();

		/* refbox log send method */
		void sendRefBoxLog();
	protected:
			msl_msgs::RefBoxCommand ref;
			map<int, msl_sensor_msgs::SharedWorldInfoPtr> shwmData;
			map<int, alica_ros_proxy::AlicaEngineInfoConstPtr> aeiData;
			mutex shwmMutex, aeiMutex;
			ros::Publisher RefereeBoxInfoBodyPublisher;
			ros::Subscriber shwmSub, aliceClientSubscriber;
			ros::NodeHandle* rosNode;
			QTcpSocket* tcpsocket;
			QUdpSocket* udpsocket;
			int counter;
			XMLProtocolParser* xmlparser;
			QTimer* sendRefBoxLogtimer;
			QTimer* sendRefBoxCmdtimer;

	};


} /* namespace rqt_pm_control */

#endif /* CNC_MSL_RQT_MSL_REFBOX_SRC_RQT_MSL_REFBOX_GAMEDATA_H_ */
