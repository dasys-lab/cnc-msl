#include "rqt_msl_refbox/RefBoxCommunication.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

namespace rqt_msl_refbox
{

	RefBoxCommunication::RefBoxCommunication(QDialog *parent)
	{
		setupUi(parent);

		tcpsocket = NULL;
		udpSocket = NULL;

		interface = "eth0";
		destHost = "172.16.1.2";
		destPort = 28097;


		Connect_btn->setText("Connect");
		Interface_val->setText(interface);
		DestHost_val->setText(destHost);
		DestPort_val->setValue(destPort);

		connected = 0;
		before_stop_gamePart = 0;

		connect(Connect_btn, SIGNAL(clicked()), this, SLOT(connectToRefBox()));
		connect(Interface_val, SIGNAL(editingFinished()), this, SLOT(SetInterface()));
		connect(DestHost_val, SIGNAL(editingFinished()), this, SLOT(SetHostAdd()));
		connect(DestPort_val, SIGNAL(valueChanged(int)), this, SLOT(SetHostPort(int)));
	}

	RefBoxCommunication::~RefBoxCommunication()
	{
		disconnect(Connect_btn, SIGNAL(clicked()), this, SLOT(connectToRefBox()));
		disconnect(DestHost_val, SIGNAL(editingFinished()), this, SLOT(SetHostAdd()));
		disconnect(DestPort_val, SIGNAL(valueChanged(int)), this, SLOT(SetHostPort(int)));
		disconnect(Interface_val, SIGNAL(editingFinished()), this, SLOT(SetInterface()));

		if (tcpsocket != NULL)
		{
			tcpsocket->close();
			delete tcpsocket;
		}

		if (udpSocket != NULL)
		{
			udpSocket->close();
			delete udpSocket;
		}
	}

	void RefBoxCommunication::connectToRefBox(void)
	{
		/* ignore if in basestation mode */
		if (RefBox_Local_btn->isChecked())
		{
			Status_val->setText("Using Local RefBox");
		}

		/* connect in old (TCP) mode if disconnected and in refbox mode */
		else if (RefBox_TCP_XML_btn->isChecked() && connected == 0)
		{
			tcpsocket = new QTcpSocket();

			Status_val->setText("Creating Socket");

			tcpsocket->connectToHost(destHost, destPort);

			if (!tcpsocket->waitForConnected(1000))
			{
				Status_val->setText("Creating Socket error");
				Connect_btn->setChecked(0);
				return;
			}

			connect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg ()));

			Status_val->setText("Connected using old protocol");
			Connect_btn->setChecked(1);
			Connect_btn->setText("Disconnect");

			connected = 1;

			Interface_val->setEnabled(0);
			DestHost_val->setEnabled(0);
			DestPort_val->setEnabled(0);
		}

		/* connect in new (UDP) mode, if disconnected and in new refbox mode */
		else if (RefBox_Multicast_XML_btn->isChecked() && connected == 0)
		{
			/* open multicast port */
			/*MulticastSocket sock;
			 if (sock.openSocket(interface.toAscii().constData(), destHost.toAscii().constData(), destPort) == -1)
			 {
			 Status_val->setText("Creating Socket error");
			 Connect_bot->setChecked(0);
			 return;
			 }

			 udpSocket = new QUdpSocket();
			 udpSocket->setSocketDescriptor(sock.getSocket(), QUdpSocket::ConnectedState, QUdpSocket::ReadOnly);

			 connect(udpSocket, SIGNAL(readyRead()) , this, SLOT(receiveRefMsg()));

			 //		udpParser = new RefBoxXML();

			 Status_val->setText("Connected using new protocol");
			 Connect_bot->setChecked(1);
			 Connect_bot->setText("Disconnect");

			 Interface_val->setEnabled(0);
			 DestHost_val->setEnabled(0);
			 DestPort_val->setEnabled(0);

			 connected = 2;*/
		}

		/* disconnect old (TCP) mode if connected in refbox mode */
		else if (connected == 1)
		{
			disconnect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsg ()));

			tcpsocket->close();
			delete tcpsocket;
			tcpsocket = NULL;

			Status_val->setText("Disconnected");
			Connect_btn->setChecked(0);
			Connect_btn->setText("Connect");

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

			Status_val->setText("Disconnected");
			Connect_btn->setChecked(0);
			Connect_btn->setText("Connect");

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

	void RefBoxCommunication::receiveRefMsg(void)
	{
		if (RefBox_Local_btn->isChecked())
		{
			return;
		}

		if (RefBox_TCP_XML_btn->isChecked())
		{
			processRefBoxMsg();
			return;
		}

		if (RefBox_Multicast_XML_btn->isChecked())
		{
			processNewRefBoxMsg();
			return;
		}
	}

	void RefBoxCommunication::processNewRefBoxMsg()
	{
	}

	void RefBoxCommunication::SetHostAdd(void)
	{
		destHost.clear();
		destHost = DestHost_val->text();
	}

	void RefBoxCommunication::SetInterface(void)
	{
		interface.clear();
		interface = Interface_val->text();
	}

	void RefBoxCommunication::SetHostPort(int val)
	{
		destPort = val;
	}

}

