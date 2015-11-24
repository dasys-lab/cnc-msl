/*
 * GameData.cpp
 *
 *  Created on: May 28, 2015
 *      Author: Stephan Opfer
 */

#include "rqt_msl_refbox/GameData.h"
#include "msl_msgs/RefBoxCommand.h"
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include "qstring.h"
#include <iostream>

namespace rqt_msl_refbox
{

	GameData::GameData(RefBox* refBox)
	{
		rosNode = new ros::NodeHandle();

		RefereeBoxInfoBodyPublisher = rosNode->advertise<msl_msgs::RefBoxCommand>("/RefereeBoxInfoBody", 2);
		shwmSub = rosNode->subscribe("/WorldModel/SharedWorldInfo", 2, &GameData::onSharedWorldmodelInfo,
										(GameData*)this);

		localToggled = false;
		xmlparser = new XMLProtocolParser(this);
		tcpToggled = false;
		multiToggled = false;
		this->refBox = refBox;
		time(&timer);
		this->counter = 0;
		this->udpsocket = nullptr;
		this->tcpsocket = nullptr;

	}

	GameData::~GameData()
	{
		if (tcpsocket != nullptr)
		{
			disconnect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg()));
			delete udpsocket;
		}
		if (udpsocket != nullptr)
		{
			disconnect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));
			delete tcpsocket;
		}
		delete rosNode;
		delete xmlparser;
	}

	void GameData::onSharedWorldmodelInfo(msl_sensor_msgs::SharedWorldInfoPtr msg)
	{
		cout << "Reveived Data" << endl;
		lock_guard<mutex> lock(this->shwmMutex);
		shwmData[msg->senderID] = msg;
	}

	void GameData::PlayOnPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendStart();
		this->refBox->lbl_command->setText("Play on");
		this->refBox->RefLog->append("Start local");
	}

	void GameData::StopPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendStop();
		this->refBox->lbl_command->setText("Stop");
		this->refBox->RefLog->append("Stop local");
	}

	void GameData::HaltPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendHalt();
		this->refBox->lbl_command->setText("Halt");
		this->refBox->RefLog->append("Halt local");
	}

	void GameData::DroppedBallPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendDroppedBall();
		this->refBox->lbl_command->setText("Dropped Ball");
		this->refBox->RefLog->append("Dropped Ball local");
	}

	void GameData::ParkingPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendParking();
		this->refBox->lbl_command->setText("Parking");
		this->refBox->RefLog->append("Parking local");
	}

	void GameData::JoystickPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::COMMAND_JOYSTICK;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
		this->refBox->lbl_command->setText("Joystick");
		this->refBox->RefLog->append("Joystick local");
	}

//================================================ Our States =======================================

	void GameData::OurKickOffPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendMagentaKickOff();
		this->refBox->lbl_command->setText("KickOff Magenta");
		this->refBox->RefLog->append("KickOff Magenta local");
	}

	void GameData::OurFreeKickPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendMagentaFreeKick();
		this->refBox->lbl_command->setText("Free Kick Magenta");
		this->refBox->RefLog->append("Free Kick Magenta local");
	}

	void GameData::OurGoalKickPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendMagentaGoalKick();
		this->refBox->lbl_command->setText("Goal Kick Magenta");
		this->refBox->RefLog->append("Goal Kick Magenta local");
	}

	void GameData::OurThrowinPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendMagentaThrownin();
		this->refBox->lbl_command->setText("Throwin Magenta");
		this->refBox->RefLog->append("Throwin Magenta local");
	}

	void GameData::OurCornerKickPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendMagentaCornerKick();
		this->refBox->lbl_command->setText("Corner Kick Magenta");
		this->refBox->RefLog->append("Corner Kick Magenta");
	}

	void GameData::OurPenaltyPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendMagentaPenalty();
		this->refBox->lbl_command->setText("Penalty Magenta");
		this->refBox->RefLog->append("Penalty Magenta local");
	}

//================================================ Their States =======================================

	void GameData::TheirKickOffPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendCyanKickOff();
		this->refBox->lbl_command->setText("KickOff Cyan");
		this->refBox->RefLog->append("KickOff Cyan local");
	}

	void GameData::TheirFreeKickPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendCyanFreeKick();
		this->refBox->lbl_command->setText("Free Kick Cyan");
		this->refBox->RefLog->append("Free Kick Cyan local");
	}

	void GameData::TheirGoalKickPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendCyanGoalKick();
		this->refBox->lbl_command->setText("Goal Kick Cyan");
		this->refBox->RefLog->append("Goal Kick Cyan local");
	}

	void GameData::TheirThrowinPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendCyanThrownin();
		this->refBox->lbl_command->setText("Throwin Cyan");
		this->refBox->RefLog->append("Throwin Cyan local");
	}

	void GameData::TheirCornerKickPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendCyanCornerKick();
		this->refBox->lbl_command->setText("Corner Kick Cyan");
		this->refBox->RefLog->append("Corner Kick Cyan local");
	}

	void GameData::TheirPenaltyPressed(void)
	{
		if (!localToggled)
		{
			return;
		}
		sendCyanPenalty();
		this->refBox->lbl_command->setText("Penalty Cyan");
		this->refBox->RefLog->append("Penalty Cyan local");
	}

	void GameData::onLocalTogled(bool checked)
	{
		this->localToggled = checked;
	}

	void GameData::onMultiTogled(bool checked)
	{
		this->multiToggled = checked;

	}

	void GameData::onTcpTogled(bool checked)
	{
		this->tcpToggled = checked;
	}

	bool GameData::isLocalToggled()
	{
		return localToggled;
	}

	bool GameData::isTcpToggled()
	{
		return tcpToggled;
	}

	bool GameData::isMultiToggled()
	{
		return multiToggled;
	}

	/*==============================  CONNECT METHODS ==============================*/

	void GameData::onConnectPressed(void)
	{
		if (counter == 1)
		{
			tcpsocket->close();
			disconnect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg()));
			delete tcpsocket;
			tcpsocket = nullptr;
		}
		if (counter == 2)
		{
			udpsocket->close();
			disconnect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));
			delete udpsocket;
			udpsocket = nullptr;
		}

		if (tcpToggled)
		{

			tcpsocket = new QTcpSocket();

			this->refBox->RefLog->append("Creating TCP Socket");

			QString destHost = this->refBox->ledit_ipaddress->text();
			quint16 destPort = this->refBox->spin_port->value();
			this->refBox->lbl_statusCon->setText("TRY CONNECT TO IP ");

			tcpsocket->connectToHost(destHost, destPort);

			if (!tcpsocket->waitForConnected(1000))
			{
				this->refBox->RefLog->append("Creating Socket TCP: error");
				this->refBox->lbl_statusCon->setText("ERROR 404");
				this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : red}");
				return;
			}

			connect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg()));

			this->refBox->lbl_statusCon->setText("TCP");
			this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : green}");
			this->counter = 1;
		}
		else if (multiToggled)
		{

			udpsocket = new QUdpSocket();
			this->refBox->RefLog->append("Creating UDP Socket");

			QString destHost = this->refBox->ledit_ipaddress->text();
			quint16 destPort = this->refBox->spin_port->value();

			this->refBox->lbl_statusCon->setText("TRY CONNECT: MILTICAST ");

			QString adressMulti = destHost;
			QHostAddress adress = QHostAddress(adressMulti);

			udpsocket->bind(adress, destPort);
			udpsocket->joinMulticastGroup(adress);

			connect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));

			this->refBox->lbl_statusCon->setText("MULTICAST");
			this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : green}");
			this->counter = 2;

		}
		else if (localToggled)
		{
			disconnect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg()));
			disconnect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));
			this->refBox->lbl_statusCon->setText("LOCAL");
			this->counter = 0;
		}
	}
	/*==============================  RECEIVE METHODS ==============================*/

	void GameData::receiveRefMsg(void)
	{
		QByteArray buffer;
		while (tcpsocket->canReadLine())
		{
			buffer = buffer + tcpsocket->readLine();
		}
		if (buffer.size() > 1)
		{
//			std::cout << "BUFFER " <<  buffer.data() << std::endl;
			tinyxml2::XMLDocument doc;
			doc.Parse(buffer.data());
			tinyxml2::XMLElement* element = doc.FirstChildElement();
			xmlparser->handle(element);
		}
	}
	void GameData::receiveRefMsgUdp(void)
	{
		QByteArray buffer;
		buffer.resize(udpsocket->pendingDatagramSize());

		QHostAddress sender;
		quint16 senderPort;

		udpsocket->readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);
//		std::cout << "BUFFER DATA: " <<  buffer.data() << std::endl;

		tinyxml2::XMLDocument doc;

		doc.Parse(buffer.data());
		tinyxml2::XMLElement* element = doc.FirstChildElement();
		xmlparser->handle(element);

	}

	/*==============================  SEND REFBOX LOG ===========================*/

	void GameData::sendRefBoxLog()
	{
		lock_guard<mutex> lock(this->shwmMutex);

		// mockup
		QString teamIntention = "Win the game";

		// general information
		QString logString = QString("{ \"type\": \"worldstate\",");
		logString += QString("\"teamName\":\"Carpe Noctem Cassel\",");
		logString += QString("\"intention\": \" " + teamIntention + "\",");

		if (this->shwmData.size() > 0)
		{
			// robots
			logString += QString("\"robots\": [");
			pair<const int, msl_sensor_msgs::SharedWorldInfoPtr> robotForObsAndBall;
			robotForObsAndBall.second->senderID = 9999999;
			for (auto robot : this->shwmData)
			{
				if (robotForObsAndBall.second->senderID > robot.second->senderID)
					robotForObsAndBall = robot;

				logString += "{\"id\": " + QString::number(robot.second->senderID, 10) + ", \"position\": ["
						+ QString::number(robot.second->odom.position.x, 'f', 3) + ","
						+ QString::number(robot.second->odom.position.y, 'f', 3) + "]," + "\"orientation\":"
						+ QString::number(robot.second->odom.position.angle, 'f', 4) + ","
						+ "\"targetPos\": [null,null,null]," + "\"velocityAng\": "
						+ QString::number(robot.second->odom.motion.rotation, 'f', 4) + "\"velocityLin\":["
						+ QString::number(robot.second->odom.motion.translation * cos(robot.second->odom.motion.angle))
						+ QString::number(robot.second->odom.motion.translation * sin(robot.second->odom.motion.angle))
						+ "]," + "\"intention\": \"null\"," + "\"batteryLevel\": null," + "\"ballEngaged\": null"
						+ "},";
			}
			// remove last comma
			logString.remove(logString.length() - 1, 1);

			// balls
			logString += QString("\"balls\": [");
			int integratedBalls = 0;
			for (auto robot : this->shwmData)
			{
				if (robot.second->ball.confidence != 0)
				{
					integratedBalls++;
					logString += QString(
							"{ \"position\": [" + QString::number(robot.second->ball.point.x, 'f', 3)
									+ QString::number(robot.second->ball.point.y, 'f', 3)
									+ QString::number(robot.second->ball.point.z, 'f', 3) + "], \"velocity\": ["
									+ QString::number(robot.second->ball.velocity.vx, 'f', 3)
									+ QString::number(robot.second->ball.velocity.vy, 'f', 3)
									+ QString::number(robot.second->ball.velocity.vz, 'f', 3) + "], \"confidence\": "
									+ QString::number(robot.second->ball.confidence, 'f', 3) + "},");
				}
				if (integratedBalls > 0)
				{
					// remove last comma
					logString.remove(logString.length() - 1, 1);
				}
			}
			logString += QString("], ");

			// obstacles
			logString += QString("\"obstacles\": [");
			int integratedObstacles = 0;
			for (auto opponents : robotForObsAndBall.second->mergedOpponents)
			{
					integratedObstacles++;
					logString += QString(
							"{ \"position\": ["
							+ QString::number(opponents.x, 'f', 3)
							+ QString::number(opponents.y, 'f', 3)
							+ "], \"velocity\": [],"
							+ "\"confidence\": null },"
							);
			}
			if (integratedObstacles > 0)
			{
				// remove last comma
				logString.remove(logString.length() - 1, 1);
			}
			logString += QString("], ");

			logString += QString("\"ageMs\": "
							+ QString::number(50, 10)
							);
		}
		else
		{
			logString += QString("\"robots\": [], \"balls\": [], \"obstacles\": [], \"agesMs\": null");
		}
		logString += "}\0";

		QByteArray tmp;
		this->tcpsocket->write(tmp.append(logString));
//
//		gettimeofday(&tv, NULL);
//		t1 = tv.tv_sec * 1000 + tv.tv_usec / 1000;
//
//		fprintf(stderr, "Worldstate Time: %d %d\n", t1 - time, t1);
	}

	/*==============================  SEND METHODS ==============================*/

	void GameData::sendCyanPenalty()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::PENALTY_CYAN;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanCornerKick()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::CORNER_CYAN;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanGoalKick()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::GOALKICK_CYAN;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanThrownin()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::THROWIN_CYAN;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanFreeKick()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::FREEKICK_CYAN;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanKickOff()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::KICKOFF_CYAN;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaPenalty()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::PENALTY_MAGENTA;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaCornerKick()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::CORNER_MAGENTA;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaGoalKick()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::GOALKICK_MAGENTA;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaThrownin()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::THROWIN_MAGENTA;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaKickOff()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::KICKOFF_MAGENTA;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaFreeKick()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::FREEKICK_MAGENTA;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendParking()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::PARK;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendDroppedBall()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::DROPBALL;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendHalt()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::HALT;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendStop()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::STOP;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendStart()
	{
		msl_msgs::RefBoxCommand ref;
		ref.cmd = msl_msgs::RefBoxCommand::START;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}
} /* namespace rqt_pm_control */
