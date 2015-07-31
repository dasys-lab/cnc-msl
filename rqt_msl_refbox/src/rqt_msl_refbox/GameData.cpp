/*
 * GameData.cpp
 *
 *  Created on: May 28, 2015
 *      Author: Stephan Opfer
 */

#include "rqt_msl_refbox/GameData.h"
#include "msl_msgs/RefereeBoxInfoBody.h"
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <iostream>

namespace rqt_msl_refbox
{

	GameData::GameData(RefBox* refBox)
	{
		rosNode = new ros::NodeHandle();

		RefereeBoxInfoBodyPublisher = rosNode->advertise<msl_msgs::RefereeBoxInfoBody>("/RefereeBoxInfoBody", 2);
		this->xmlparser = new XMLProtocolParser(this);
		localToggled = false;
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
		if(tcpsocket != nullptr)
		{
			disconnect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg()));
			delete udpsocket;
		}
		if(udpsocket != nullptr)
		{
			disconnect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));
			delete tcpsocket;
		}
		delete rosNode;
		delete xmlparser;

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
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::command_joystick;
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
		if(counter == 1)
		{
			tcpsocket->close();
			disconnect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg()));
			delete tcpsocket;
			tcpsocket = nullptr;
		}
		if(counter == 2)
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

//			QString destHost = this->refBox->ledit_ipaddress->text();
			QString destHost = "141.51.122.174";
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

			this->refBox->lbl_statusCon->setText("CONNECTED");
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

			QString adressMulti = "230.0.0.1";
			QHostAddress adress = QHostAddress(adressMulti);

			udpsocket->bind(adress, destPort);
			udpsocket->joinMulticastGroup(adress);

			connect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));

			this->refBox->lbl_statusCon->setText("CONNECTED MULTICAST");
			this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : green}");
			this->counter = 2;

		}
		else if (localToggled)
		{
			disconnect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg()));
			disconnect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));
			this->refBox->lbl_statusCon->setText("LOCAL MODE");
			this->counter = 0;
		}
	}
	/*==============================  RECEIVE METHODS ==============================*/

	void GameData::receiveRefMsg(void)
	{
		QByteArray buffer;
		qint64 _packetSize = tcpsocket->bytesAvailable();
		buffer = tcpsocket->read(_packetSize);

		std::cout << buffer.data() << std::endl;
	}
	void GameData::receiveRefMsgUdp(void)
	{
		QByteArray buffer;
		buffer.resize(udpsocket->pendingDatagramSize());

		QHostAddress sender;
		quint16 senderPort;

		udpsocket->readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);
		std::cout << "BUFFER DATA: " <<  buffer.data() << std::endl;

		tinyxml2::XMLDocument doc;

		doc.Parse(buffer.data());
		tinyxml2::XMLElement* element = doc.FirstChildElement();
		xmlparser->handle(element);

	}

	/*==============================  SEND METHODS ==============================*/

	void GameData::sendCyanPenalty()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::penaltyCyan;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanCornerKick()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::cornerCyan;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanGoalKick()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::goalkickCyan;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanThrownin()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::throwinCyan;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanFreeKick()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::freekickCyan;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendCyanKickOff()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::kickoffCyan;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaPenalty()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::penaltyMagenta;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaCornerKick()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::cornerMagenta;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaGoalKick()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::goalkickMagenta;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaThrownin()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::throwinMagenta;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaKickOff()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::kickoffMagenta;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendMagentaFreeKick()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::freekickMagenta;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendParking()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::park;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendDroppedBall()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::droppedBall;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendHalt()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::halt;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendStop()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::stop;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}

	void GameData::sendStart()
	{
		msl_msgs::RefereeBoxInfoBody ref;
		ref.lastCommand = msl_msgs::RefereeBoxInfoBody::start;
		this->RefereeBoxInfoBodyPublisher.publish(ref);
	}
} /* namespace rqt_pm_control */
