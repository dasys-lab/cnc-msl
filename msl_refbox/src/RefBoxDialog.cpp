/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA BASESTATION
 *
 * CAMBADA BASESTATION is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA BASESTATION is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "RefBoxDialog.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "MulticastSocket.h"

RefBoxDialog::RefBoxDialog(QDialog *parent)
{
	setupUi(parent);

	/* Conecção com a refBox */
	socket = NULL;
	udpSocket = NULL;

#if 0
	destHost = "172.16.39.10";
	destPort = 28097;
	interface = "lo";
	destHost = "224.16.32.75";
	destPort = 30000;
	interface = "lo";
	destHost = "230.0.0.1";
	destPort = 30000;
#endif
	interface = "eth0";
	destHost = "172.16.1.2";
	destPort = 28097;

	/* inicializações */
	Connect_bot->setText("Connect");
	Interface_val->setText(interface);
	DestHost_val->setText(destHost);
	DestPort_val->setValue(destPort);

	connected = 0;

	before_stop_gamePart = 0;

	/* Conecções */
	connect(Connect_bot, SIGNAL(clicked()), this, SLOT(connectToHost()));
	connect(Interface_val, SIGNAL(editingFinished()), this, SLOT(SetInterface()));
	connect(DestHost_val, SIGNAL(editingFinished()), this, SLOT(SetHostAdd()));
	connect(DestPort_val, SIGNAL(valueChanged(int)), this, SLOT(SetHostPort(int)));
	connect(apply_Button, SIGNAL(clicked()), this, SLOT(apply_Button_pressed()));

	connect(timmer_start_bot, SIGNAL(clicked()), this, SLOT(Timer_start_bot_pressed()));
//	connect(timmer_stop_bot, SIGNAL(clicked()), this , SLOT(Timer_stop_bot_pressed()));
}

RefBoxDialog::~RefBoxDialog()
{
	disconnect(Connect_bot, SIGNAL(clicked()), this, SLOT(connectToHost()));
	disconnect(DestHost_val, SIGNAL(editingFinished()), this, SLOT(SetHostAdd()));
	disconnect(DestPort_val, SIGNAL(valueChanged(int)), this, SLOT(SetHostPort(int)));
	disconnect(apply_Button, SIGNAL(clicked()), this, SLOT(apply_Button_pressed()));
	disconnect(timmer_start_bot, SIGNAL(clicked()), this, SLOT(Timer_start_bot_pressed()));
//	disconnect(timmer_stop_bot, SIGNAL(clicked()), this , SLOT(Timer_stop_bot_pressed()));

	if (socket != NULL)
	{
		socket->close();
		delete socket;
	}

	if (udpSocket != NULL)
	{
		udpSocket->close();
		delete udpSocket;
//		delete udpParser;
	}
}

void RefBoxDialog::connectToHost(void)
{
	/* ignore if in basestation mode */
	if (Basestation_Bot->isChecked())
	{
		Status_val->setText("Basestation mode");
	}

	/* connect in old (TCP) mode if disconnected and in refbox mode */
	else if (RefBox_Bot->isChecked() && connected == 0)
	{
		socket = new QTcpSocket();

		/* Criação do socket */
		Status_val->setText("Creating Socket");

		socket->connectToHost(destHost, destPort);

		if (!socket->waitForConnected(1000))
		{
			Status_val->setText("Creating Socket error");
			Connect_bot->setChecked(0);
			return;
		}

		progressBar->setValue(50);

		connect(socket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg ()));

		progressBar->setValue(100);

		Status_val->setText("Connected using old protocol");
		Connect_bot->setChecked(1);
		Connect_bot->setText("Disconnect");

		connected = 1;

		Interface_val->setEnabled(0);
		DestHost_val->setEnabled(0);
		DestPort_val->setEnabled(0);
	}

	/* connect in new (UDP) mode, if disconnected and in new refbox mode */
	else if (NewRefBox_Bot->isChecked() && connected == 0)
	{
		/* open multicast port */
		MulticastSocket sock;
		if (sock.openSocket(interface.toAscii().constData(), destHost.toAscii().constData(), destPort) == -1)
		{
			Status_val->setText("Creating Socket error");
			Connect_bot->setChecked(0);
			return;
		}

		udpSocket = new QUdpSocket();
		udpSocket->setSocketDescriptor(sock.getSocket(), QUdpSocket::ConnectedState, QUdpSocket::ReadOnly);

		connect(udpSocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg()));

		Status_val->setText("Connected using new protocol");
		Connect_bot->setChecked(1);
		Connect_bot->setText("Disconnect");

		Interface_val->setEnabled(0);
		DestHost_val->setEnabled(0);
		DestPort_val->setEnabled(0);

		connected = 2;
	}

	/* disconnect old (TCP) mode if connected in refbox mode */
	else if (connected == 1)
	{
		disconnect(socket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg ()));

		socket->close();
		delete socket;
		socket = NULL;

		progressBar->setValue(0);
		Status_val->setText("Disconnected");
		Connect_bot->setChecked(0);
		Connect_bot->setText("Connect");

		Interface_val->setEnabled(1);
		DestHost_val->setEnabled(1);
		DestPort_val->setEnabled(1);

		connected = 0;
	}

	/* connect in new (UDP) mode if disconnected and in new refbox mode */
	else if (connected == 2)
	{
		disconnect(udpSocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg ()));

		udpSocket->close();
		delete udpSocket;
		udpSocket = NULL;
//		delete udpParser;

		Status_val->setText("Disconnected");
		Connect_bot->setChecked(0);
		Connect_bot->setText("Connect");

		Interface_val->setEnabled(1);
		DestHost_val->setEnabled(1);
		DestPort_val->setEnabled(1);

		connected = 0;
	}

	else
	{
		Status_val->setText("Ignored");
	}
}

void RefBoxDialog::receiveRefMsg(void)
{
	if (Basestation_Bot->isChecked())
	{
		return;
	}

	if (RefBox_Bot->isChecked())
	{
		processRefBoxMsg();
		return;
	}

	if (NewRefBox_Bot->isChecked())
	{
		processNewRefBoxMsg();
		return;
	}
}

void RefBoxDialog::processNewRefBoxMsg()
{
	/*	printf("\nA refbox message received\n");
	 printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");

	 char msg[1500];
	 qint64 nread = udpSocket->read(msg, 1500-1);
	 msg[nread] = '\0';
	 printf("%s", msg);
	 printf("---------------------------\n");
	 bool retv = udpParser->parse(msg, db_coach_info);
	 printf("---------------------------\n");
	 printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
	 if (retv == true) emit transmitCoach();*/
}

void RefBoxDialog::processRefBoxMsg()
{
	// TODO
	WSColor teamColor = Blue;

	QString Cmd("SsNkKpPfFgGtTcCHaAL");
	char msg[1500];
	QString valid_cmd;
	qint64 nread = 0;

	memset(msg, 0, 1500);

	/* Leitura */
	nread = socket->read(msg, 1500 - 1);
	printf("Ref box Message -> %s\n", msg);
	QString Msg(msg);

	/* Comandos Internos */
	if (Msg.contains("Welcome.."))
	{
		/* Mensagem inicial, não precisa de processar */
		printf("Ref Box connected\n");
		Msg.remove("Welcome..");
	}

	if (Msg.contains("Hh") || Msg.contains("h"))
	{
		printf("Message contains Hh or h!\n");

		Msg.remove("h");
		Msg.remove("Hh");
	}

	if (Msg.contains("He"))
	{
		printf("Message contains He!\n");

		Msg.remove("He");
	}

	if (Msg.contains('e'))
	{
		printf("Message contains e!\n");

		Msg.remove("e");
	}

	if (Msg.contains('1'))
	{
		printf("Message contains 1!\n");

		Msg.remove("1");
	}

	if (Msg.contains('2'))
	{
		printf("Message contains 2!\n");

		Msg.remove("2");

	}

	if (Msg.length() < 0)
	{
		printf("Message length < 0!\n");
		return;
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
				printf("SIGstart\n");

			if (valid_cmd == "S")
				printf("SIGstop\n");

			if (((valid_cmd == "K") && (teamColor == Cyan)) || ((valid_cmd == "k") && (teamColor == Magenta)))
				printf("SIGourKickOff\n");

			if (((valid_cmd == "k") && (teamColor == Cyan)) || ((valid_cmd == "K") && (teamColor == Magenta)))
				printf("SIGtheirKickOff\n");

			if (((valid_cmd == "P") && (teamColor == Cyan)) || ((valid_cmd == "p") && (teamColor == Magenta)))
				printf("SIGourPenalty\n");

			if (((valid_cmd == "p") && (teamColor == Cyan)) || ((valid_cmd == "P") && (teamColor == Magenta)))
				printf("SIGtheirPenalty\n");

			if (((valid_cmd == "F") && (teamColor == Cyan)) || ((valid_cmd == "f") && (teamColor == Magenta)))
				printf("SIGourFreeKick\n");

			if (((valid_cmd == "f") && (teamColor == Cyan)) || ((valid_cmd == "F") && (teamColor == Magenta)))
				printf("SIGtheirFreeKick\n");

			if (((valid_cmd == "G") && (teamColor == Cyan)) || ((valid_cmd == "g") && (teamColor == Magenta)))
				printf("SIGourGoalKick\n");

			if (((valid_cmd == "g") && (teamColor == Cyan)) || ((valid_cmd == "G") && (teamColor == Magenta)))
				printf("SIGtheirGoalKick\n");

			if (((valid_cmd == "T") && (teamColor == Cyan)) || ((valid_cmd == "t") && (teamColor == Magenta)))
				printf("SIGourThrowIn\n");

			if (((valid_cmd == "t") && (teamColor == Cyan)) || ((valid_cmd == "T") && (teamColor == Magenta)))
				printf("SIGtheirThrowIn\n");

			if (((valid_cmd == "C") && (teamColor == Cyan)) || ((valid_cmd == "c") && (teamColor == Magenta)))
				printf("SIGourCornerKick\n");

			if (((valid_cmd == "c") && (teamColor == Cyan)) || ((valid_cmd == "C") && (teamColor == Magenta)))
				printf("SIGtheirCornerKick\n");

			if (((valid_cmd == "A") && (teamColor == Cyan)) || ((valid_cmd == "a") && (teamColor == Magenta)))
				printf("OwnGoal++\n");

			if (((valid_cmd == "a") && (teamColor == Cyan)) || ((valid_cmd == "A") && (teamColor == Magenta)))
				printf("TheirGoal++\n");

			if (valid_cmd == "N")
				printf("SIGdropBall\n");

			if (valid_cmd == "L")
				printf("SIGparking\n");

			printf("emit TransmitCoach()\n");
			Q_EMIT transmitCoach();
		}
	}
}

void RefBoxDialog::SetHostAdd(void)
{
	destHost.clear();
	destHost = DestHost_val->text();
}

void RefBoxDialog::SetInterface(void)
{
	interface.clear();
	interface = Interface_val->text();
}

void RefBoxDialog::SetHostPort(int val)
{
	destPort = val;
}

void RefBoxDialog::update_manual_config(void)
{
	// TODO
	//cambada_goals_sb->setValue(1); // own goals
	//mf_goals_sb->setValue(2); // their goals

	// TODO
	printf("Set the game period \"setCurrentIndes(X)\" to 2, 1, or 0\n");
//	if (db_coach_info->GamePart == 2)
//	{
//		game_period_cb->setCurrentIndex(2);
//	}
//	else if (db_coach_info->GamePart == 1)
//	{
//		game_period_cb->setCurrentIndex(1);
//	}
//	else if (db_coach_info->GamePart == 0)
//	{
//		game_period_cb->setCurrentIndex(0);
//	}

}

void RefBoxDialog::apply_Button_pressed(void)
{
	int new_gamepart = 0;

	printf("Set Coach our goals and their goals");
	/*db_coach_info->Coach_Info.ourGoals = cambada_goals_sb->value();
	db_coach_info->Coach_Info.theirGoals = mf_goals_sb->value();*/

	if (game_period_cb->currentIndex() == 0)
	{
		new_gamepart = 0;
	}
	else if (game_period_cb->currentIndex() == 1)
	{
		new_gamepart = 1;
	}
	else if (game_period_cb->currentIndex() == 2)
	{
		new_gamepart = 2;
	}

	//TODO
	/*if (db_coach_info->GamePart != new_gamepart)
	{
		db_coach_info->gameStartTime = time(NULL);
	}*/
	printf("emit transmitCoach()");
	Q_EMIT transmitCoach();

	printf("emit updateGameParam()");
	Q_EMIT updateGameParam();
}

void RefBoxDialog::Timer_start_bot_pressed(void)
{
	// TODO
	printf("Set some timing information about the game!");
	/*
	if (db_coach_info != NULL)
	{
		db_coach_info->GameTime.restart();
		int min = this->StartMin->text().toInt();
		int sec = this->StartSec->text().toInt();
		db_coach_info->gTimeSecOffset = min * 60 + sec;
		db_coach_info->logTimeOffset = 0;

		db_coach_info->gameStartTime = time(NULL);
	}*/
	Q_EMIT updateGameParam();
}

void RefBoxDialog::Timer_stop_bot_pressed(void)
{
}

