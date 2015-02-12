#ifndef __REFBOXDIALOG_H
#define __REFBOXDIALOG_H

#include <QTcpSocket>
#include <QUdpSocket>
#include <QSocketNotifier>

#include "ui_RefBoxCommunication.h"

namespace rqt_msl_refbox
{

	class RefBoxCommunication : public QDialog, public Ui::RefBoxCommunication
	{
	Q_OBJECT

	public:
		RefBoxCommunication(QDialog *parent = 0);
		~RefBoxCommunication();
		int connected;

		void processNewRefBoxMsg();
		void processRefBoxMsg();

	private:

	protected:
		QString destHost;
		quint16 destPort;
		QString interface;

		/* TCP Socket */
		QTcpSocket *socket;
		QUdpSocket *udpSocket;
		//QSocketNotifier *notifier;

		char data_received[1500];
		int before_stop_gamePart;

	public slots:
		void connectToHost(void);
		void receiveRefMsg(void);

		void SetInterface(void);
		void SetHostAdd(void);
		void SetHostPort(int val);

		void update_manual_config(void);
		void apply_Button_pressed(void);

		void Timer_start_bot_pressed(void);
		void Timer_stop_bot_pressed(void);

	signals:
		void transmitCoach(void);
		void changeGoalColor(int);
		void updateGameParam(void);

	};
}

#endif
