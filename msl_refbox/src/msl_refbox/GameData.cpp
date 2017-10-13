#include "msl_refbox/GameData.h"
#include "msl_msgs/RefBoxCommand.h"

#include <msl/robot/IntRobotID.h>

#include <QString>
#include <chrono>
#include <iostream>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <sstream>

namespace msl_refbox
{

GameData::GameData(RefBox *refBox)
{
    rosNode = new ros::NodeHandle();

    RefereeBoxInfoBodyPublisher = rosNode->advertise<msl_msgs::RefBoxCommand>("/RefereeBoxInfoBody", 2);
    shwmSub = rosNode->subscribe("/WorldModel/SharedWorldInfo", 2, &GameData::onSharedWorldmodelInfo, (GameData *)this);
    aliceClientSubscriber = rosNode->subscribe("/AlicaEngine/AlicaEngineInfo", 2, &GameData::onAlicaEngineInfo, (GameData *)this);

    localToggled = false;
    xmlparser = new XMLProtocolParser(this);
    this->tcpToggled = true;
    this->udpToggled = false;
    this->charToggled = true;
    this->xmlToggled = false;
    this->reconnectToggled = true;
    this->refBox = refBox;
    this->counter = 0;
    this->udpsocket = nullptr;
    this->tcpsocket = nullptr;
    this->sendRefBoxLogtimer = new QTimer();
    connect(sendRefBoxLogtimer, SIGNAL(timeout()), this, SLOT(sendRefBoxLog()));
    this->sendRefBoxLogtimer->start(100);

    this->sendRefBoxCmdtimer = new QTimer();
    connect(sendRefBoxCmdtimer, SIGNAL(timeout()), this, SLOT(sendRefBoxCmd()));
    this->sendRefBoxCmdtimer->start(333);

    this->reconnectTimer = new QTimer();
    connectionState = DISCONNECTED;
    this->goalsCyan = 0;
    this->goalsMagenta = 0;
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
    auto itraei = this->aeiData.begin();
    while (itraei != this->aeiData.end())
    {
        auto keyCopy = itraei->first;
        itraei = this->aeiData.erase(itraei);
        delete keyCopy;
    }
    auto itrshwm = this->shwmData.begin();
    while (itrshwm != this->shwmData.end())
    {
        auto keyCopy = itrshwm->first;
        itrshwm = this->shwmData.erase(itrshwm);
        delete keyCopy;
    }
    auto itrdate = this->date.begin();
    while (itrdate != this->date.end())
    {
        auto keyCopy = itrdate->first;
        itrdate = this->date.erase(itrdate);
        delete keyCopy;
    }
    delete sendRefBoxLogtimer;
    delete rosNode;
    delete xmlparser;
    delete sendRefBoxCmdtimer;
}

void GameData::sendRefBoxCmd()
{
    if (this->localToggled == false && (tcpsocket != nullptr || udpsocket != nullptr) && this->ref.cmd != 0)
        this->RefereeBoxInfoBodyPublisher.publish(ref);
}

void normalizeAngle(double &ang)
{
    while (ang <= -M_PI)
    {
        ang += 2 * M_PI;
    }
    while (ang > M_PI)
    {
        ang -= 2 * M_PI;
    }
}

void GameData::onSharedWorldmodelInfo(msl_sensor_msgs::SharedWorldInfoPtr msg)
{
    cout << "Received Data" << endl;
    double tmp = msg->ball.point.x;
    msg->ball.point.x = -msg->ball.point.y / 1000.0;
    msg->ball.point.y = tmp / 1000.0;
    msg->ball.point.z = msg->ball.point.z / 1000.0;
    tmp = msg->ball.velocity.vx;
    msg->ball.velocity.vx = msg->ball.velocity.vy / 1000.0;
    msg->ball.velocity.vy = tmp / 1000.0;
    msg->ball.velocity.vz /= 1000.0;

    tmp = msg->odom.position.x;
    msg->odom.position.x = -msg->odom.position.y / 1000.0;
    msg->odom.position.y = tmp / 1000.0;

    if (msg->odom.position.angle > M_PI)
        msg->odom.position.angle -= M_PI;
    else
        msg->odom.position.angle += M_PI;

    msg->odom.motion.translation /= 1000.0;

    tmp = msg->negotiatedBall.point.x;
    msg->negotiatedBall.point.x = -msg->negotiatedBall.point.y / 1000.0;
    msg->negotiatedBall.point.y = tmp / 1000.0;
    msg->negotiatedBall.point.z = msg->negotiatedBall.point.z / 1000.0;
    tmp = msg->negotiatedBall.velocity.vx;
    msg->negotiatedBall.velocity.vx = msg->negotiatedBall.velocity.vy / 1000.0;
    msg->negotiatedBall.velocity.vy = tmp / 1000.0;
    msg->negotiatedBall.velocity.vz /= 1000.0;

    tmp = msg->sharedBall.point.x;
    msg->sharedBall.point.x = -msg->sharedBall.point.y / 1000.0;
    msg->sharedBall.point.y = tmp / 1000.0;
    msg->sharedBall.point.z = msg->sharedBall.point.z / 1000.0;
    tmp = msg->sharedBall.velocity.vx;
    msg->sharedBall.velocity.vx = msg->sharedBall.velocity.vy / 1000.0;
    msg->sharedBall.velocity.vy = tmp / 1000.0;
    msg->sharedBall.velocity.vz /= 1000.0;

    for (int i = 0; i < msg->path.size(); i++)
    {
        tmp = msg->path.at(i).x;
        msg->path.at(i).x = msg->path.at(i).y / 1000.0;
        msg->path.at(i).y = tmp / 1000.0;
    }

    for (int i = 0; i < msg->mergedOpponents.size(); i++)
    {
        tmp = msg->mergedOpponents.at(i).x;
        msg->mergedOpponents.at(i).x = msg->mergedOpponents.at(i).y / 1000.0;
        msg->mergedOpponents.at(i).y = tmp / 1000.0;
    }

    for (int i = 0; i < msg->mergedTeamMembers.size(); i++)
    {
        tmp = msg->mergedTeamMembers.at(i).x;
        msg->mergedTeamMembers.at(i).x = msg->mergedTeamMembers.at(i).y / 1000.0;
        msg->mergedTeamMembers.at(i).y = tmp / 1000.0;
    }

    for (int i = 0; i < msg->obstacles.size(); i++)
    {
        tmp = msg->obstacles.at(i).x;
        msg->obstacles.at(i).x = msg->obstacles.at(i).y / 1000.0;
        msg->obstacles.at(i).y = tmp / 1000.0;
    }

    lock_guard<mutex> lock(this->shwmMutex);
    auto sender = factory.create(msg->senderID.id);
    this->date[sender] = std::chrono::system_clock::now();
    shwmData[sender] = msg;
}

void GameData::onAlicaEngineInfo(alica_ros_proxy::AlicaEngineInfoConstPtr aei)
{
    lock_guard<mutex> lock(this->aeiMutex);
    auto sender = factory.create(aei->senderID.id);
    aeiData[sender] = aei;
}

void GameData::PlayOnPressed(void)
{
    sendStart(true);
}

void GameData::StopPressed(void)
{
    sendStop(true);
}

void GameData::HaltPressed(void)
{
    sendHalt(true);
}

void GameData::DroppedBallPressed(void)
{
    sendDroppedBall(true);
}

void GameData::ParkingPressed(void)
{
    sendParking(true);
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

    this->log("Joystick", true, Side::ALL);
}

//================================================ Our States =======================================

void GameData::OurKickOffPressed(void)
{
    sendMagentaKickOff(true);
}

void GameData::OurFreeKickPressed(void)
{
    sendMagentaFreeKick(true);
}

void GameData::OurGoalKickPressed(void)
{
    sendMagentaGoalKick(true);
}

void GameData::OurThrowinPressed(void)
{
    sendMagentaThrownin(true);
}

void GameData::OurCornerKickPressed(void)
{
    sendMagentaCornerKick(true);
}

void GameData::OurPenaltyPressed(void)
{
    sendMagentaPenalty(true);
}

//================================================ Their States =======================================

void GameData::TheirKickOffPressed(void)
{
    sendCyanKickOff(true);
}

void GameData::TheirFreeKickPressed(void)
{
    sendCyanFreeKick(true);
}

void GameData::TheirGoalKickPressed(void)
{
    sendCyanGoalKick(true);
}

void GameData::TheirThrowinPressed(void)
{
    sendCyanThrowin(true);
}

void GameData::TheirCornerKickPressed(void)
{
    sendCyanCornerKick(true);
}

void GameData::TheirPenaltyPressed(void)
{
    sendCyanPenalty(true);
}

void GameData::onLocalToggled(bool checked)
{
    if (checked)
        this->refBox->btn_connect->setEnabled(false);
    else
        this->refBox->btn_connect->setEnabled(true);

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

void GameData::onReconnectToggled(bool checked)
{
    this->reconnectToggled = checked;
}

/*==============================  CONNECT METHODS ==============================*/

void GameData::onConnectPressed(void)
{
    switch (connectionState)
    {
    case DISCONNECTED: // Button == Connect
        connectNet();
        break;
    case TCP_CONNECTED: // Button == Disconnect
        disconnectTCP();
        break;
    case UDP_CONNECTED: // Button == Disconnect
        disconnectUDP();
        break;
    case RECONNECTING: // Button == Abort
        connectionState = DISCONNECTED;
        reconnectTimer->stop();
        disconnect(reconnectTimer, SIGNAL(timeout()), this, SLOT(connectNet()));
        this->refBox->btn_connect->setText("Connect");
        break;
    default:
        break;
    }
}

void GameData::connectNet()
{
    const QString host = this->refBox->ledit_ipaddress->text();
    const quint16 port = this->refBox->spin_port->value();

    if (localToggled)
    {
        disconnect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgTcp()));
        disconnect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));
        this->refBox->lbl_statusCon->setText("LOCAL");
    }
    else if (tcpToggled)
    {
        tcpsocket = connectTCP(host, port);

        if (tcpsocket == nullptr)
        {
            this->refBox->RefLog->append("Creating Socket TCP: error");
            this->refBox->lbl_statusCon->setText("TCP Connection Error");
            this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : red}");
        }
        else
        {
            connectionState = TCP_CONNECTED;
            this->refBox->btn_connect->setText("Disconnect");

            this->refBox->lbl_statusCon->setText("TCP");
            this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : green}");

            connect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgTcp()));
            connect(tcpsocket, SIGNAL(disconnected()), this, SLOT(onTcpDisconnected()));

            reconnectTimer->stop();
            disconnect(reconnectTimer, SIGNAL(timeout()), this, SLOT(connectNet()));
        }
    }
    else if (udpToggled)
    {
        udpsocket = connectUDP(host, port);

        if (udpsocket == nullptr)
        {
            this->refBox->RefLog->append("Creating Socket UDP: error");
            this->refBox->lbl_statusCon->setText("UDP Connection Error");
            this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : red}");
        }
        else
        {
            this->connectionState = UDP_CONNECTED;
            this->refBox->btn_connect->setText("Disconnect");

            connect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));

            this->refBox->lbl_statusCon->setText("UDP");
            this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : green}");
        }
    }
}

void GameData::disconnectTCP()
{
    connectionState = DISCONNECTING;

    disconnect(tcpsocket, SIGNAL(disconnected()), this, SLOT(onTcpDisconnected()));

    tcpsocket->close();
    delete tcpsocket;
    tcpsocket = nullptr;

    connectionState = DISCONNECTED;

    this->refBox->btn_connect->setText("Connect");

    this->refBox->lbl_statusCon->setText("Disconnected");
    this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : red}");
}

void GameData::disconnectUDP()
{
    connectionState = DISCONNECTING;

    udpsocket->close();
    disconnect(udpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgUdp()));
    delete udpsocket;
    udpsocket = nullptr;

    connectionState = DISCONNECTED;

    this->refBox->btn_connect->setText("Connect");

    this->refBox->lbl_statusCon->setText("Disconnected");
    this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : red}");
}

QTcpSocket *GameData::connectTCP(QString host, qint16 port)
{
    QTcpSocket *tcpsocket = new QTcpSocket();

    tcpsocket->connectToHost(host, port);

    if (!tcpsocket->waitForConnected(1000))
        return nullptr;

    return tcpsocket;
}

QUdpSocket *GameData::connectUDP(QString host, qint16 port)
{
    QUdpSocket *udpsocket = new QUdpSocket();
    this->refBox->RefLog->append("Creating UDP Socket");
    this->refBox->lbl_statusCon->setText("TRY CONNECT: UDP ");

    const QHostAddress address = QHostAddress(host);

    udpsocket->bind(address, port);
    udpsocket->joinMulticastGroup(address);

    return udpsocket;
}

/*==============================  RECEIVE METHODS ==============================*/
void GameData::receiveRefMsgTcp(void)
{
    if (xmlToggled)
    {
        buffer.append(this->tcpsocket->readLine());
        int index = buffer.indexOf('\n');

        if (index > 0)
        {
            this->parseXML(buffer.left(index));
            buffer.remove(0, index + 1);
            this->updateGoals();
        }
    }
    else if (charToggled)
    {
        char msg[4096];
        int size = tcpsocket->read(msg, 4096);

        this->refBox->debugLog("Number of received characters by tcp: " + std::to_string(size));
        if (size > 0)
        {
            if (size < 4096)
                msg[size] = '\0';
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
        if (xmlToggled)
        {
            this->parseXML(buffer);
        }
        else if (charToggled)
        {
            processCharacterBasedProtocol(buffer.data());
        }
    }
}

void GameData::parseXML(const QByteArray &data)
{
    if (this->localToggled)
        return;

    tinyxml2::XMLDocument doc;
    doc.Parse(data);
    tinyxml2::XMLElement *element = doc.FirstChildElement();

    if (element == nullptr)
    {
        std::cerr << "Received bad refbox message: " << buffer.constData() << std::endl;
    }
    else
    {
        xmlparser->handle(element);
    }
}

void GameData::updateGoals()
{
    std::string goal = std::to_string(this->goalsCyan) + " : " + std::to_string(this->goalsMagenta);
    this->refBox->lbl_score->setText(QString::fromStdString(goal));
    this->ref.goalsCyan = this->goalsCyan;
    this->ref.goalsMagenta = this->goalsMagenta;
}

void GameData::setGoals(Side side, int value)
{
    if (side == Side::CYAN)
    {
        this->goalsCyan = value;
    }
    else if (side == Side::MAGENTA)
    {
        this->goalsMagenta = value;
    }
}

int GameData::getGoals(Side side)
{
    if (side == Side::CYAN)
    {
        return this->goalsCyan;
    }
    else if (side == Side::MAGENTA)
    {
        return this->goalsMagenta;
    }

    return -1;
}

void GameData::processCharacterBasedProtocol(const char *data)
{
    if (this->localToggled)
        return;

    QString Cmd("SsNkKpPfFgGtTcCHaALDdoOiIyYrR");
    QString valid_cmd;

    QString Msg(data);

    if (Msg.size() > 10)
    {
        printf("Received a to long message from refbox: %s\n", data);

        return;
    }

    this->refBox->debugLog(Msg.toStdString());

    /* Comandos Internos */
    if (Msg.contains("W"))
    {
        // printf("Ref Box connected\n");
        Msg.remove("W");
    }

    if (Msg.contains("h"))
    {
        this->sendStop(false);
        Msg.remove("h");
        this->ref.gameStage.Stage = msl_msgs::RefBoxCommand::HALFTIME;
        this->log("End first half", false, GameData::Side::ALL);
    }

    if (Msg.contains('e'))
    {
        this->sendStop(false);
        Msg.remove("e");
        this->ref.gameStage.Stage = msl_msgs::RefBoxCommand::END_GAME;
        this->log("End second half", false, GameData::Side::ALL);
    }

    if (Msg.contains('1'))
    {
        // Start First Half
        this->setGoals(Side::CYAN, 0);
        this->setGoals(Side::MAGENTA, 0);
        this->ref.gameStage.Stage = msl_msgs::RefBoxCommand::FIRST_HALF;
        this->log("Begin first half", false, GameData::Side::ALL);
        Msg.remove("1");
    }

    if (Msg.contains('2'))
    {
        // Start Second half
        Msg.remove("2");
        this->ref.gameStage.Stage = msl_msgs::RefBoxCommand::SECOND_HALF;
        this->log("Begin second half", false, GameData::Side::ALL);
    }

    if (Msg.contains('3'))
    {
        // Third half
        Msg.remove("3");
        this->log("Begin third half", false, GameData::Side::ALL);
    }

    if (Msg.contains('4'))
    {
        // Fourth half
        Msg.remove("4");
        this->log("Begin fourth half", false, GameData::Side::ALL);
    }

    /* Proc msg */
    valid_cmd.clear();
    for (int i = 0; i < Msg.length(); i++)
    {
        if (Cmd.contains(Msg[i])) //é um comando válido??
        {
            // Cmd válido
            valid_cmd = Msg[i];
            // printf("last valid cmd-> %c \n", Msg[i]);

            if (valid_cmd == "s")
                this->sendStart(false);

            if (valid_cmd == "S")
                this->sendStop(false);

            if (valid_cmd == "K")
                this->sendCyanKickOff(false);

            if (valid_cmd == "k")
                this->sendMagentaKickOff(false);

            if (valid_cmd == "P")
                this->sendCyanPenalty(false);

            if (valid_cmd == "p")
                this->sendMagentaPenalty(false);

            if (valid_cmd == "F")
                this->sendCyanFreeKick(false);

            if (valid_cmd == "f")
                this->sendMagentaFreeKick(false);

            if (valid_cmd == "G")
                this->sendCyanGoalKick(false);

            if (valid_cmd == "g")
                this->sendMagentaGoalKick(false);

            if (valid_cmd == "T")
                this->sendCyanThrowin(false);

            if (valid_cmd == "t")
                this->sendMagentaThrownin(false);

            if (valid_cmd == "C")
                this->sendCyanCornerKick(false);

            if (valid_cmd == "c")
                this->sendMagentaCornerKick(false);

            if (valid_cmd == "A")
            {
                this->setGoals(Side::CYAN, this->getGoals(Side::CYAN) + 1);
                this->log("Goal", false, GameData::Side::CYAN);
                this->updateGoals();
            }

            if (valid_cmd == "a")
            {
                this->setGoals(Side::MAGENTA, this->getGoals(Side::MAGENTA) + 1);
                this->log("Goal", false, GameData::Side::MAGENTA);
                this->updateGoals();
            }

            if (valid_cmd == "D")
            {
                this->setGoals(Side::CYAN, this->getGoals(Side::CYAN) - 1);
                this->log("Goal removed", false, GameData::Side::CYAN);
                this->updateGoals();
            }

            if (valid_cmd == "d")
            {
                this->setGoals(Side::MAGENTA, this->getGoals(Side::MAGENTA) - 1);
                this->log("Goal removed", false, GameData::Side::MAGENTA);
                this->updateGoals();
            }

            if (valid_cmd == "N")
                this->sendDroppedBall(false);

            if (valid_cmd == "L")
                this->sendParking(false);

            if (valid_cmd == "O")
                this->log("Player out", false, Side::CYAN);

            if (valid_cmd == "o")
                this->log("Player out", false, Side::MAGENTA);

            if (valid_cmd == "I")
                this->log("Player in", false, Side::CYAN);

            if (valid_cmd == "i")
                this->log("Player in", false, Side::MAGENTA);

            if (valid_cmd == "Y")
                this->log("Yellow Card", false, Side::CYAN);

            if (valid_cmd == "y")
                this->log("Yellow Card", false, Side::MAGENTA);

            if (valid_cmd == "R")
                this->log("Red Card", false, Side::CYAN);

            if (valid_cmd == "r")
                this->log("Red Card", false, Side::MAGENTA);
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
        bool robotFound = false;
        const msl::robot::IntRobotID *tmpID = nullptr;
        for (auto robot : this->shwmData)
        {
            auto now = std::chrono::system_clock::now();
            auto sender = factory.create(robot.second->senderID.id);
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - date[sender]).count() > 1000)
            {
                continue;
            }

            stringstream ss;
            ss << sender;
            logString += "{\"id\": " + QString(ss.str().c_str()) + ", \"pose\": [" + QString().sprintf("%.3f", robot.second->odom.position.x) + "," +
                         QString().sprintf("%.3f", robot.second->odom.position.y) + "," + QString::number(robot.second->odom.position.angle, 'f', 4) + "]," +
                         "\"targetPos\": [null,null,null]," + "\"velocity\":[" +
                         QString::number(robot.second->odom.motion.translation * cos(robot.second->odom.motion.angle), 'f', 3) + "," +
                         QString::number(robot.second->odom.motion.translation * sin(robot.second->odom.motion.angle), 'f', 3) + "," +
                         QString::number(robot.second->odom.motion.rotation, 'f', 4) + QString("], \"intention\": \"");
            auto iter = this->aeiData.find(sender);
            if (iter != this->aeiData.end())
            {
                logString += QString(iter->second->currentTask.c_str()) + " - " + QString(iter->second->currentPlan.c_str()) + " - " +
                             QString(iter->second->currentState.c_str()) + "\",";
            }
            else
            {
                logString += QString("\",");
            }
            logString += QString("\"batteryLevel\": null, \"ballEngaged\": null },");
            if (tmpID == nullptr || tmpID > sender)
            {
                delete tmpID;
                tmpID = sender;
                robotFound = true;
                robotForObs = robot.second;
            }
            else
            {
                delete sender;
            }
        }
        delete tmpID;
        if (!robotFound)
        {
            return;
        }
        // remove last comma
        logString.remove(logString.length() - 1, 1);
        logString += "]";

        // balls
        logString += QString(",\"balls\": [");
        int integratedBalls = 0;
        for (auto robot : this->shwmData)
        {
            auto now = chrono::system_clock::now();
            auto tmp = factory.create(robot.second->senderID.id);
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - date[tmp]).count() > 1000)
            {
                continue;
            }
            if (robot.second->ball.confidence != 0)
            {
                integratedBalls++;
                logString += QString("{ \"position\": [" + QString::number(robot.second->ball.point.x, 'f', 3) + "," +
                                     QString::number(robot.second->ball.point.y, 'f', 3) + "," + QString::number(robot.second->ball.point.z, 'f', 3) +
                                     "], \"velocity\": [" + QString::number(robot.second->ball.velocity.vx, 'f', 3) + "," +
                                     QString::number(robot.second->ball.velocity.vy, 'f', 3) + "," + QString::number(robot.second->ball.velocity.vz, 'f', 3) +
                                     "], \"confidence\": " + QString::number(robot.second->ball.confidence, 'f', 3) + "},");
            }
            delete tmp;
        }
        if (integratedBalls > 0)
        {
            // remove last comma
            logString.remove(logString.length() - 1, 1);
        }
        logString += QString("], ");

        // obstacles
        logString += QString("\"obstacles\": [");
        int integratedObstacles = 0;
        for (auto opponents : robotForObs->mergedOpponents)
        {
            integratedObstacles++;
            logString += QString("{ \"position\": [" + QString::number(opponents.x, 'f', 3) + "," + QString::number(opponents.y, 'f', 3) +
                                 "], \"velocity\": [null, null]," + "\"confidence\": null },");
        }
        if (integratedObstacles > 0)
        {
            // remove last comma
            logString.remove(logString.length() - 1, 1);
        }
        logString += QString("], ");

        logString += QString("\"ageMs\": " + QString::number(50, 10));
    }
    else
    {
        logString += QString(",\"robots\": [], \"balls\": [], \"obstacles\": [], \"ageMs\": 50");
    }
    logString += "}\0";

    if (this->tcpsocket == nullptr || !this->tcpsocket->isValid() || this->tcpsocket->state() != QAbstractSocket::ConnectedState)
        return;

    QByteArray tmp;
    tmp.append(logString);
    tmp.append('\0');
    if (this->tcpsocket->write(tmp) < 0)
    {
        cout << "Error: " << this->tcpsocket->errorString().toStdString() << endl;
    }
    // cout << endl << "SENDING" << endl;
}

/*==============================  SEND METHODS ==============================*/

void GameData::sendCyanPenalty(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::PENALTY_CYAN;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Penalty", local, Side::CYAN);
}

void GameData::sendCyanCornerKick(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::CORNER_CYAN;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Corner Kick", local, Side::CYAN);
}

void GameData::sendCyanGoalKick(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::GOALKICK_CYAN;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Goal Kick", local, Side::CYAN);
}

void GameData::sendCyanThrowin(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::THROWIN_CYAN;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Throw-in", local, Side::CYAN);
}

void GameData::sendCyanFreeKick(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::FREEKICK_CYAN;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Free Kick", local, Side::CYAN);
}

void GameData::sendCyanKickOff(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::KICKOFF_CYAN;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Kick Off", local, Side::CYAN);
}

void GameData::sendMagentaPenalty(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::PENALTY_MAGENTA;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Penalty", local, Side::MAGENTA);
}

void GameData::sendMagentaCornerKick(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::CORNER_MAGENTA;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Corner Kick", local, Side::MAGENTA);
}

void GameData::sendMagentaGoalKick(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::GOALKICK_MAGENTA;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Goal Kick", local, Side::MAGENTA);
}

void GameData::sendMagentaThrownin(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::THROWIN_MAGENTA;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Throw-in", local, Side::MAGENTA);
}

void GameData::sendMagentaKickOff(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::KICKOFF_MAGENTA;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Kick Off", local, Side::MAGENTA);
}

void GameData::sendMagentaFreeKick(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::FREEKICK_MAGENTA;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Free Kick", local, Side::MAGENTA);
}

void GameData::sendParking(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::PARK;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Parking", local, Side::ALL);
}

void GameData::sendDroppedBall(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::DROPBALL;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Drop Ball", local, Side::ALL);
}

void GameData::sendHalt(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::HALT;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Halt", local, Side::ALL);
}

void GameData::sendStop(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::STOP;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Stop", local, Side::ALL);
}

void GameData::sendStart(bool local)
{
    if ((local && !localToggled) || (!local && localToggled))
    {
        return;
    }

    ref.cmd = msl_msgs::RefBoxCommand::START;
    this->RefereeBoxInfoBodyPublisher.publish(ref);

    this->log("Start", local, Side::ALL);
}

void GameData::log(std::string msg, bool local, Side side)
{
    std::time_t t = std::time(NULL);
    char mbstr[100];
    std::strftime(mbstr, sizeof(mbstr), "%T", std::localtime(&t));

    logStream.str("");
    logStream << mbstr;
    logStream << " ";
    std::string sideStr;

    switch (side)
    {
    case ALL:
        logStream << "\t\t";
        sideStr = "";
        break;
    case MAGENTA:
        logStream << " MAGENTA\t";
        sideStr = "Magenta ";
        break;
    case CYAN:
        logStream << " CYAN\t";
        sideStr = "Cyan ";
        break;
    }

    logStream << msg;

    if (local)
    {
        this->refBox->lbl_command->setText(QString(std::string(sideStr + msg + " (local)").c_str()));
        logStream << " local";
    }
    else
    {
        this->refBox->lbl_command->setText(QString(std::string(sideStr + msg).c_str()));
    }

    this->refBox->RefLog->append(QString(logStream.str().c_str()));
}

void GameData::onTcpDisconnected()
{
    disconnect(tcpsocket, SIGNAL(disconnected()), this, SLOT(onTcpDisconnected()));
    disconnect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receiveRefMsgTcp()));

    this->refBox->RefLog->append("Disconnected from Refreebox");
    this->refBox->lbl_statusCon->setText("Disconnected");
    this->refBox->lbl_statusCon->setStyleSheet("QLabel { background-color : red}");

    this->refBox->btn_connect->setText("Connect");

    if (connectionState == DISCONNECTING || connectionState == RECONNECTING)
        return;

    if (!reconnectToggled)
        return;

    // Enable Auto reconnecting
    connectionState = RECONNECTING;
    this->refBox->lbl_statusCon->setText("Reconnecting...");
    connect(reconnectTimer, SIGNAL(timeout()), this, SLOT(connectNet()));
    this->reconnectTimer->start(500);

    this->refBox->btn_connect->setText("Abort");
}

} /* namespace msl_refbox */
