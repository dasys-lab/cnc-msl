#include <msl_control/Robot.h>

#include <process_manager/RobotExecutableRegistry.h>
#include <ros/ros.h>

#include "ui_ControlledMSLRobot.h"
#include <QFrame>

#include <chrono>
#include <limits.h>
#include <msl_control/MSLControl.h>

using std::string;

namespace msl_control
{
Robot::Robot(string robotName, int robotId, MSLControl *parentMSLControl)
    : RobotMetaData(robotName, robotId)
    , parentMSLControl(parentMSLControl)
    , widget(new QFrame())
    , uiMSLRobot(new Ui::MSLRobotWidget())
    , shown(true)
{
    this->uiMSLRobot->setupUi(this);

    // manual configuration of widgets
    this->uiMSLRobot->robotNameLbl->setText(
        QString(std::string(this->name + " (" + std::to_string(robotId) + ")").c_str()));

    this->parentMSLControl->robotControlWidget_.robotControlLayout->addWidget(this);
}

Robot::~Robot()
{
}

QSize Robot::sizeHint()
{
    return QSize(500, 500);
}

void Robot::updateGUI(std::chrono::system_clock::time_point now)
{
	if ((now - this->timeLastMsgReceived) > std::chrono::milliseconds(1000))
	{
		this->uiMSLRobot->KickVolValue->setText("");
		this->uiMSLRobot->BallPossValue->setText("");
	}
}

void Robot::toggle()
{
    if (shown)
    {
        this->hide();
    }
    else
    {
        this->show();
    }
}

void Robot::show()
{
    this->shown = true;
    QFrame::show();
}

void Robot::hide()
{
    this->shown = false;
    QFrame::hide();
}

void Robot::handleKickerStatInfo(
    std::pair<std::chrono::system_clock::time_point, msl_actuator_msgs::KickerStatInfoPtr> timeKSIpair)
{
    this->timeLastMsgReceived = timeKSIpair.first;
    this->uiMSLRobot->BallPossValue->setText(QString::number(timeKSIpair.second->capVoltage));
}

void Robot::handleSharedWorldInfo(
    std::pair<std::chrono::system_clock::time_point, msl_sensor_msgs::SharedWorldInfoPtr> timeSWIpair)
{
    this->timeLastMsgReceived = timeSWIpair.first;
    if (timeSWIpair.second->ballPossessionStatus == msl_sensor_msgs::SharedWorldInfo::HAVE_BALL)
    {
        this->uiMSLRobot->BallPossValue->setText(QString("HaveBall"));
    }
    else if (timeSWIpair.second->ballPossessionStatus == msl_sensor_msgs::SharedWorldInfo::NO_BALL_SEEN)
    {
    	this->uiMSLRobot->BallPossValue->setText(QString("NoBallSeen"));
    }
    else if (timeSWIpair.second->ballPossessionStatus == msl_sensor_msgs::SharedWorldInfo::ASIDE_OF_KICKER)
    {
    	this->uiMSLRobot->BallPossValue->setText(QString("ASideOfKicker"));
    }
    else if (timeSWIpair.second->ballPossessionStatus == msl_sensor_msgs::SharedWorldInfo::NOT_IN_KICKER_DISTANCE)
    {
    	this->uiMSLRobot->BallPossValue->setText(QString("NotInKickerDistance"));
    }
    else if (timeSWIpair.second->ballPossessionStatus == msl_sensor_msgs::SharedWorldInfo::LIGHT_BARRIER_UNBLOCKED)
    {
    	this->uiMSLRobot->BallPossValue->setText(QString("LightBarrierUnblocked"));
    }
    else
    {
    	this->uiMSLRobot->BallPossValue->setText(QString("Unknown"));
    }
}

} /* namespace msl_control */
