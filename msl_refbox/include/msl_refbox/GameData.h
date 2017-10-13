#pragma once

#include "alica_ros_proxy/AlicaEngineInfo.h"
#include "msl_msgs/RefBoxCommand.h"
#include "msl_refbox/RefBox.h"
#include "msl_refbox/XMLProtocolParser.h"
#include "msl_refbox/tinyxml2.h"
#include "msl_sensor_msgs/SharedWorldInfo.h"

#include <supplementary/IAgentID.h>
#include <msl/robot/IntRobotIDFactory.h>

#include "ros/ros.h"
#include <QFile>
#include <QTcpSocket>
#include <QTimer>
#include <QUdpSocket>
#include <QtGui>
#include <chrono>
#include <map>
#include <mutex>

namespace msl
{
namespace robot
{
	class IntRobotID;
}
}

namespace msl_refbox
{
class RefBox;
class XMLProtocolParser;
class GameData : public QObject
{
    Q_OBJECT
  public:
    void onSharedWorldmodelInfo(msl_sensor_msgs::SharedWorldInfoPtr msg);
    void onAlicaEngineInfo(alica_ros_proxy::AlicaEngineInfoConstPtr aei);
    void processCharacterBasedProtocol(const char *data);

    void sendCyanCornerKick(bool local);
    void sendCyanThrowin(bool local);
    void sendStart(bool local);
    void sendStop(bool local);
    void sendHalt(bool local);
    void sendDroppedBall(bool local);
    void sendParking(bool local);
    void sendMagentaKickOff(bool local);
    void sendMagentaFreeKick(bool local);
    void sendMagentaGoalKick(bool local);
    void sendMagentaThrownin(bool local);
    void sendMagentaCornerKick(bool local);
    void sendMagentaPenalty(bool local);
    void sendCyanKickOff(bool local);
    void sendCyanFreeKick(bool local);
    void sendCyanGoalKick(bool local);
    void sendCyanPenalty(bool local);

  public:
    enum Side
    {
        ALL,
        CYAN,
        MAGENTA
    };

    GameData(RefBox *refBox);
    virtual ~GameData();
    RefBox *refBox;
    bool localToggled;
    bool udpToggled;
    bool tcpToggled;
    bool xmlToggled;
    bool charToggled;
    bool reconnectToggled;

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
    void onReconnectToggled(bool checked);

    void onConnectPressed(void);
    void receiveRefMsgTcp(void);
    void receiveRefMsgUdp(void);
    void onDisconnectPressed(void);

    void sendRefBoxCmd(void);

    void onTcpDisconnected();
    void connectNet();

    /* refbox log send method */
    void sendRefBoxLog();
    void log(std::string method, bool local, Side side);
    void updateGoals();
    void setGoals(Side, int value);
    int getGoals(Side);

  protected:
    enum ConnectionState
    {
        DISCONNECTING,
        DISCONNECTED,
        RECONNECTING,
        TCP_CONNECTED,
        UDP_CONNECTED,
    } connectionState;

    msl::robot::IntRobotIDFactory factory;
    msl_msgs::RefBoxCommand ref;
    std::map<const msl::robot::IntRobotID *, msl_sensor_msgs::SharedWorldInfoPtr, supplementary::IAgentIDComparator> shwmData;
    std::map<const msl::robot::IntRobotID *, alica_ros_proxy::AlicaEngineInfoConstPtr, supplementary::IAgentIDComparator> aeiData;
    std::map<const msl::robot::IntRobotID *, std::chrono::system_clock::time_point, supplementary::IAgentIDComparator> date;
    std::mutex shwmMutex, aeiMutex;
    ros::Publisher RefereeBoxInfoBodyPublisher;
    ros::Subscriber shwmSub, aliceClientSubscriber;
    ros::NodeHandle *rosNode;
    QTcpSocket *tcpsocket;
    QUdpSocket *udpsocket;
    int counter;
    XMLProtocolParser *xmlparser;
    QTimer *sendRefBoxLogtimer;
    QTimer *sendRefBoxCmdtimer;
    QTimer *reconnectTimer;
    std::stringstream logStream;
    QByteArray buffer;
    int goalsMagenta;
    int goalsCyan;

    QTcpSocket *connectTCP(QString host, qint16 port);
    QUdpSocket *connectUDP(QString host, qint16 port);

    void disconnectTCP();
    void disconnectUDP();
    void parseXML(const QByteArray &data);
};

} /* namespace rqt_pm_control */
