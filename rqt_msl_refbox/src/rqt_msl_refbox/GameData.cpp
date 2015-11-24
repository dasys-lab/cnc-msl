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
		aliceClientSubscriber = rosNode->subscribe("/AlicaEngine/AlicaEngineInfo", 2, &GameData::onAlicaEngineInfo,
													(GameData*)this);

		localToggled = false;
		xmlparser = new XMLProtocolParser(this);
		this->tcpToggled = false;
		this->udpToggled = false;
		this->charToggled = false;
		this->xmlToggled = false;
		this->refBox = refBox;
		this->counter = 0;
		this->udpsocket = nullptr;
		this->tcpsocket = nullptr;
		this->sendRefBoxLogtimer = new QTimer();
		connect(sendRefBoxLogtimer, SIGNAL(timeout()), this, SLOT(sendRefBoxLog()));
		this->sendRefBoxLogtimer->start(100);

		this->sendRefBoxCmdtimer = new QTimer();
		connect (sendRefBoxCmdtimer, SIGNAL(timeout()), this, SLOT(sendRefBoxCmd()));
		this->sendRefBoxCmdtimer->start(333);
	}

	GameData::~GameData()
	{
		if (tcpsocket != nullptr)
		{
			disconnect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgTcp()));
			delete udpsocket;
		}
		if (udpsocket != nullptr)
		{
			disconnect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));
			delete tcpsocket;
		}
		delete sendRefBoxLogtimer;
		delete rosNode;
		delete xmlparser;
		delete sendRefBoxCmdtimer;
		delete sendRefBoxLogtimer;
	}

	void GameData::sendRefBoxCmd()
	{
		if(tcpsocket != nullptr || udpsocket != nullptr)
			this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::onSharedWorldmodelInfo(msl_sensor_msgs::SharedWorldInfoPtr msg)
	{
		cout << "Received Data" << endl;
		double tmp = msg->ball.point.x;
		msg->ball.point.x = -msg->ball.point.y / 1000.0;
		msg->ball.point.y = tmp / 1000.0;
		msg->ball.point.z = msg->ball.point.z / 1000.0;
		tmp = msg->ball.velocity.vx;
		msg->ball.velocity.vx = msg->ball.velocity.vy/1000.0;
		msg->ball.velocity.vy = tmp/1000.0;
		msg->ball.velocity.vz /= 1000.0;

		tmp = msg->odom.position.x;
		msg->odom.position.x = -msg->odom.position.y/1000.0;
		msg->odom.position.y = tmp/1000.0;
		msg->odom.position.angle -= 3.14159265/2.0;
		msg->odom.motion.angle -= 3.14159265/2.0;
		msg->odom.motion.translation /= 1000.0;

		tmp = msg->negotiatedBall.point.x;
		msg->negotiatedBall.point.x = -msg->negotiatedBall.point.y/1000.0;
		msg->negotiatedBall.point.y = tmp/1000.0;
		msg->negotiatedBall.point.z = msg->negotiatedBall.point.z/1000.0;
		tmp = msg->negotiatedBall.velocity.vx;
		msg->negotiatedBall.velocity.vx = msg->negotiatedBall.velocity.vy/1000.0;
		msg->negotiatedBall.velocity.vy = tmp/1000.0;
		msg->negotiatedBall.velocity.vz /= 1000.0;

		tmp = msg->sharedBall.point.x;
		msg->sharedBall.point.x = -msg->sharedBall.point.y/1000.0;
		msg->sharedBall.point.y = tmp/1000.0;
		msg->sharedBall.point.z = msg->sharedBall.point.z/1000.0;
		tmp = msg->sharedBall.velocity.vx;
		msg->sharedBall.velocity.vx = msg->sharedBall.velocity.vy/1000.0;
		msg->sharedBall.velocity.vy = tmp/1000.0;
		msg->sharedBall.velocity.vz /= 1000.0;


		for(int i=0; i<msg->path.size(); i++) {
			tmp = msg->path.at(i).x;
			msg->path.at(i).x = msg->path.at(i).y/1000.0;
			msg->path.at(i).y = tmp/1000.0;
		}

		for(int i=0; i<msg->mergedOpponents.size(); i++) {
			tmp = msg->mergedOpponents.at(i).x;
			msg->mergedOpponents.at(i).x = msg->mergedOpponents.at(i).y/1000.0;
			msg->mergedOpponents.at(i).y = tmp/1000.0;
		}

		for(int i=0; i<msg->mergedTeamMembers.size(); i++) {
			tmp = msg->mergedTeamMembers.at(i).x;
			msg->mergedTeamMembers.at(i).x = msg->mergedTeamMembers.at(i).y/1000.0;
			msg->mergedTeamMembers.at(i).y = tmp/1000.0;
		}

		for(int i=0; i<msg->obstacles.size(); i++) {
			tmp = msg->obstacles.at(i).x;
			msg->obstacles.at(i).x = msg->obstacles.at(i).y/1000.0;
			msg->obstacles.at(i).y = tmp/1000.0;
		}

		lock_guard<mutex> lock(this->shwmMutex);
		shwmData[msg->senderID] = msg;
	}

	void GameData::onAlicaEngineInfo(alica_ros_proxy::AlicaEngineInfoConstPtr aei)
	{
		lock_guard<mutex> lock(this->aeiMutex);
		aeiData[aei->senderID] = aei;
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

	void GameData::onLocalToggled(bool checked)
	{
		this->localToggled = checked;
	}

	void GameData::onUdpToggled(bool checked)
	{
		this->udpToggled = checked;
	}

	void GameData::onTcpToggled(bool checked)
	{
		this->tcpToggled = checked;
	}

	void GameData::onXmlToggled(bool checked)
	{
		this->xmlToggled = checked;
	}

	void GameData::onCharToggled(bool checked)
	{
		this->charToggled = checked;
	}

	/*==============================  CONNECT METHODS ==============================*/

	void GameData::onConnectPressed(void)
	{
		if (counter == 1)
		{
			tcpsocket->close();
			disconnect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgTcp()));
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

		if (localToggled)
		{
			disconnect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgTcp()));
			disconnect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));
			this->refBox->lbl_statusCon->setText("LOCAL");
			this->counter = 0;
		}
		else if (tcpToggled)
		{
			tcpsocket = new QTcpSocket();

			this->refBox->RefLog->append("Creating TCP Socket");

			QString destHost = this->refBox->ledit_ipaddress->text();
			quint16 destPort = this->refBox->spin_port->value();
			this->refBox->lbl_statusCon->setText("TRY CONNECT TO IP ");

			tcpsocket->connectToHost(destHost, destPort);
			this->refBox->lbl_statusCon->setText("TRY CONNECT: TCP ");
			if (!tcpsocket->waitForConnected(1000))
			{
				this->refBox->RefLog->append("Creating Socket TCP: error");
				this->refBox->lbl_statusCon->setText("ERROR 404");
				this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : red}");
				return;
			}

			connect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgTcp()));

			this->refBox->lbl_statusCon->setText("TCP");
			this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : green}");
			this->counter = 1;
		}
		else if (udpToggled)
		{

			udpsocket = new QUdpSocket();
			this->refBox->RefLog->append("Creating UDP Socket");

			QString destHost = this->refBox->ledit_ipaddress->text();
			quint16 destPort = this->refBox->spin_port->value();

			this->refBox->lbl_statusCon->setText("TRY CONNECT: UDP ");

			QString adressMulti = destHost;
			QHostAddress adress = QHostAddress(adressMulti);

			udpsocket->bind(adress, destPort);
			udpsocket->joinMulticastGroup(adress);

			connect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));

			this->refBox->lbl_statusCon->setText("UDP");
			this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : green}");
			this->counter = 2;

		}

	}

	/*==============================  RECEIVE METHODS ==============================*/
	void GameData::receiveRefMsgTcp(void)
	{
		if (!localToggled && xmlToggled)
		{
			QByteArray buffer;
			buffer = buffer + this->tcpsocket->readLine();
			if (buffer.size() > 0)
			{
				tinyxml2::XMLDocument doc;
				doc.Parse(buffer.data());
				tinyxml2::XMLElement* element = doc.FirstChildElement();
				xmlparser->handle(element);
			}
		}
		else if (!localToggled && charToggled)
		{
			char msg[4096];
			int size = tcpsocket->read(msg,4096);
			if (size > 0)
			{
				processCharacterBasedProtocol(msg);
			}
		}
	}

	void GameData::receiveRefMsgUdp(void)
	{
		QByteArray buffer;
		buffer.resize(udpsocket->pendingDatagramSize());

		QHostAddress sender;
		quint16 senderPort;

		udpsocket->readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);

		if (buffer.size() > 0)
		{
			if (!localToggled && xmlToggled)
			{
				tinyxml2::XMLDocument doc;
				doc.Parse(buffer.data());
				tinyxml2::XMLElement* element = doc.FirstChildElement();
				xmlparser->handle(element);
			}
			else if (!localToggled && charToggled)
			{
				processCharacterBasedProtocol(buffer.data());
			}
		}

	}

	void GameData::processCharacterBasedProtocol(const char * data) {
			QString Cmd("SsNkKpPfFgGtTcCHaALDd");
			QString valid_cmd;

			printf("Ref box Message -> %s\n", data);
			QString Msg(data);

			/* Comandos Internos */
			if (Msg.contains("W"))
			{
				printf("Ref Box connected\n");
				Msg.remove("W");
			}

			if (Msg.contains("h"))
			{
				this->sendStop();
				Msg.remove("h");
			}

			if (Msg.contains('e'))
			{
				this->sendStop();
				Msg.remove("e");
			}

			if (Msg.contains('1'))
			{
				//Start First Half
				ref.goalsCyan = 0;
				ref.goalsMagenta = 0;
				Msg.remove("1");
			}

			if (Msg.contains('2'))
			{
				//Start Second half
				Msg.remove("2");

			}
			/*******************************************************Filipe**************/
			if (Msg.contains('3'))
			{
				//Third half
				Msg.remove("3");
			}

			if (Msg.contains('4'))
			{
				//Fourth half
				Msg.remove("4");
			}

			/* Proc msg */
			valid_cmd.clear();
			for (int i = 0; i < Msg.length(); i++)
			{
				if (Cmd.contains(Msg[i])) //é um comando válido??
				{
					//Cmd válido
					valid_cmd = Msg[i];
					//printf("last valid cmd-> %c \n", Msg[i]);

					if (valid_cmd == "s")
						this->sendStart();

					if (valid_cmd == "S")
						this->sendStop();

					if (valid_cmd == "K")
						this->sendCyanKickOff();

					if (valid_cmd == "k")
						this->sendMagentaKickOff();

					if (valid_cmd == "P")
						this->sendCyanPenalty();

					if (valid_cmd == "p")
						this->sendMagentaPenalty();

					if (valid_cmd == "F")
						this->sendCyanFreeKick();

					if (valid_cmd == "f")
						this->sendMagentaFreeKick();

					if (valid_cmd == "G")
						this->sendCyanGoalKick();

					if (valid_cmd == "g")
						this->sendMagentaGoalKick();

					if (valid_cmd == "T")
						this->sendCyanThrownin();

					if (valid_cmd == "t")
						this->sendMagentaThrownin();

					if (valid_cmd == "C")
						this->sendCyanCornerKick();

					if (valid_cmd == "c")
						this->sendMagentaCornerKick();

					if (valid_cmd == "A")
						ref.goalsCyan++;

					if (valid_cmd == "a")
						ref.goalsMagenta++;

					if (valid_cmd == "D")
						ref.goalsCyan--;

					if (valid_cmd == "d")
						ref.goalsMagenta--;
					if (valid_cmd == "N")
						this->sendDroppedBall();

					if (valid_cmd == "L")
						this->sendParking();
				}
			}
		}

	/*==============================  SEND REFBOX LOG ===========================*/

	void GameData::sendRefBoxLog()
	{
//		if (this->tcpsocket == nullptr
//				|| !this->tcpsocket->isValid()
//				|| this->tcpsocket->state() != QAbstractSocket::ConnectingState)
//			return;

		lock_guard<mutex> lock(this->shwmMutex);

		// mockup
		QString teamIntention = "Win the game";

		// general information
		QString logString = QString("{ \"type\": \"worldstate\",");
		logString += QString("\"teamName\":\"CNC\",");
		logString += QString("\"intention\": \"" + teamIntention + "\"");
		if (this->shwmData.size() > 0)
		{
			// robots
			logString += QString(",\"robots\": [");
			msl_sensor_msgs::SharedWorldInfoPtr robotForObs;
			int sID = 9999999;
			for (auto robot : this->shwmData)
			{
				if (sID > robot.second->senderID) {
					sID = robot.second->senderID;
					robotForObs = robot.second;
				}
				logString += "{\"id\": " + QString::number(robot.second->senderID, 10) + ", \"position\": ["
						+ QString::number(robot.second->odom.position.x, 'f', 3) + ","
						+ QString::number(robot.second->odom.position.y, 'f', 3) + "]," + "\"orientation\":"
						+ QString::number(robot.second->odom.position.angle, 'f', 4) + ","
						+ "\"targetPos\": [null,null,null]," + "\"velocityAng\": "
						+ QString::number(robot.second->odom.motion.rotation, 'f', 4) + ",\"velocityLin\":["
						+ QString::number(robot.second->odom.motion.translation * cos(robot.second->odom.motion.angle), 'f', 3)
						+ ","
						+ QString::number(robot.second->odom.motion.translation * sin(robot.second->odom.motion.angle), 'f', 3)
						+ QString("], \"intention\": \"");
				auto iter = this->aeiData.find(robot.second->senderID);
				if (iter != this->aeiData.end())
				{
					logString += QString(iter->second->currentTask.c_str()) + " - " + QString(iter->second->currentPlan.c_str()) + " - " + QString(iter->second->currentState.c_str()) + "\",";
				}
				else
				{
					logString += QString("\",");
				}
				logString += QString("\"batteryLevel\": null, \"ballEngaged\": null },");

			}
			// remove last comma
			logString.remove(logString.length() - 1, 1);
			logString += "]";

			// balls
			logString += QString(",\"balls\": [");
			int integratedBalls = 0;
			for (auto robot : this->shwmData)
			{
				if (robot.second->ball.confidence != 0)
				{
					integratedBalls++;
					logString += QString(
							"{ \"position\": [" + QString::number(robot.second->ball.point.x, 'f', 3) + ","
									+ QString::number(robot.second->ball.point.y, 'f', 3) + ","
									+ QString::number(robot.second->ball.point.z, 'f', 3) + "], \"velocity\": ["
									+ QString::number(robot.second->ball.velocity.vx, 'f', 3) + ","
									+ QString::number(robot.second->ball.velocity.vy, 'f', 3) + ","
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
			for (auto opponents : robotForObs->mergedOpponents)
			{
					integratedObstacles++;
					logString += QString(
							"{ \"position\": ["
							+ QString::number(opponents.x, 'f', 3)
							+ ","
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
			logString += QString(",\"robots\": [], \"balls\": [], \"obstacles\": [], \"agesMs\": null");
		}
		logString += "}\0";

		cout << logString.toStdString() << endl;

		if (this->tcpsocket == nullptr
						|| !this->tcpsocket->isValid()
						|| this->tcpsocket->state() != QAbstractSocket::ConnectedState)
					return;

		QByteArray tmp;
		tmp.append(logString);
		tmp.append('\0');
		if(this->tcpsocket->write(tmp) < 0){
			cout << "Error: " << this->tcpsocket->errorString().toStdString() << endl;
		}
		//cout << endl << "SENDING" << endl;
	}

	/*==============================  SEND METHODS ==============================*/

	void GameData::sendCyanPenalty()
	{
		ref.cmd = msl_msgs::RefBoxCommand::PENALTY_CYAN;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanCornerKick()
	{
		ref.cmd = msl_msgs::RefBoxCommand::CORNER_CYAN;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanGoalKick()
	{
		ref.cmd = msl_msgs::RefBoxCommand::GOALKICK_CYAN;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanThrownin()
	{
		ref.cmd = msl_msgs::RefBoxCommand::THROWIN_CYAN;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanFreeKick()
	{
		ref.cmd = msl_msgs::RefBoxCommand::FREEKICK_CYAN;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanKickOff()
	{
		ref.cmd = msl_msgs::RefBoxCommand::KICKOFF_CYAN;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaPenalty()
	{
		ref.cmd = msl_msgs::RefBoxCommand::PENALTY_MAGENTA;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaCornerKick()
	{
		ref.cmd = msl_msgs::RefBoxCommand::CORNER_MAGENTA;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaGoalKick()
	{
		ref.cmd = msl_msgs::RefBoxCommand::GOALKICK_MAGENTA;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaThrownin()
	{
		ref.cmd = msl_msgs::RefBoxCommand::THROWIN_MAGENTA;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaKickOff()
	{
		ref.cmd = msl_msgs::RefBoxCommand::KICKOFF_MAGENTA;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaFreeKick()
	{
		ref.cmd = msl_msgs::RefBoxCommand::FREEKICK_MAGENTA;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendParking()
	{
		ref.cmd = msl_msgs::RefBoxCommand::PARK;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendDroppedBall()
	{
		ref.cmd = msl_msgs::RefBoxCommand::DROPBALL;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendHalt()
	{
		ref.cmd = msl_msgs::RefBoxCommand::HALT;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendStop()
	{
		ref.cmd = msl_msgs::RefBoxCommand::STOP;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendStart()
	{
		ref.cmd = msl_msgs::RefBoxCommand::START;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}
} /* namespace rqt_pm_control */
