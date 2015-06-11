/*
 * GameData.h
 *
 *  Created on: May 28, 2015
 *      Author: Stephan Opfer
 */

#ifndef CNC_MSL_RQT_MSL_REFBOX_SRC_RQT_MSL_REFBOX_GAMEDATA_H_
#define CNC_MSL_RQT_MSL_REFBOX_SRC_RQT_MSL_REFBOX_GAMEDATA_H_

#include "ros/ros.h"
#include <QFile>
#include <QtGui>
#include <rqt_msl_refbox/RefBox.h>
#include <time.h>
#include <QTcpSocket>
#include <QUdpSocket>

namespace rqt_msl_refbox
{
	class RefBox;
	class GameData : public QObject
	{
		Q_OBJECT
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
		bool isLocalToggled();
		bool isMultiToggled();
		bool isTcpToggled();
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
		void onLocalTogled(bool checked);
		void onMultiTogled(bool checked);
		void onTcpTogled(bool checked);

		void onConnectPressed(void);
		void receiveRefMsg(void);
	protected:
			ros::Publisher RefereeBoxInfoBodyPublisher;
			ros::NodeHandle* rosNode;
			bool localToggled;
			bool multiToggled;
			bool tcpToggled;
			RefBox* refBox;
			time_t timer;
			QTcpSocket* tcpsocket;
	};


} /* namespace rqt_pm_control */

#endif /* CNC_MSL_RQT_MSL_REFBOX_SRC_RQT_MSL_REFBOX_GAMEDATA_H_ */
