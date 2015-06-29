#ifndef __REFBOXDIALOG_H
#define __REFBOXDIALOG_H

#include <QTcpSocket>
#include <QUdpSocket>
#include <QSocketNotifier>


namespace rqt_msl_refbox
{

	class RefBoxCommunication : public QDialog
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


		QTcpSocket *tcpsocket;/**< TCP Socket for point-to-point connection*/

		QUdpSocket *udpSocket;/**< UDP Socket for multi-cast connection */

		char data_received[1500];
		int before_stop_gamePart;

	public Q_SLOTS:
		void connectToRefBox(void);
		void receiveRefMsg(void);

		void SetInterface(void);
		void SetHostAdd(void);
		void SetHostPort(int val);

	Q_SIGNALS:
		void transmitCoach(void);
		void changeGoalColor(int);
		void updateGameParam(void);

	};
}

#endif
