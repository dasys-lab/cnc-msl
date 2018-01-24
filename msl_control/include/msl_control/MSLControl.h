#pragma once

#include <rqt_gui_cpp/plugin.h>

#include "ros/ros.h"
#include <msl_control/Robot.h>
#include <ros/macros.h>

#include <msl_actuator_msgs/KickerStatInfo.h>
#include <msl_sensor_msgs/SharedWorldInfo.h>

#include <QDialog>
#include <QWidget>
#include <QtGui>
#include <ui_MSLControl.h>

#include <chrono>
#include <mutex>
#include <queue>
#include <utility>

namespace supplementary
{
class SystemConfig;
class RobotExecutableRegistry;
}

namespace msl_control
{

class MSLControl : public rqt_gui_cpp::Plugin
{

    Q_OBJECT

  public:
    MSLControl();
    virtual void initPlugin(qt_gui_cpp::PluginContext &context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);

    void addRobot();
    void removeRobot();

    static std::chrono::duration<double> msgTimeOut;

    Ui::RobotControlWidget robotControlWidget_;
    QWidget *widget_;

    supplementary::RobotExecutableRegistry *pmRegistry;
    ros::NodeHandle *rosNode;

  private:
    ros::Subscriber kickerStatInfoSub;
    ros::Subscriber sharedWorldInfoSub;

    supplementary::SystemConfig *sc;

    std::map<int, Robot *> controlledRobotsMap;
    std::queue<std::pair<std::chrono::system_clock::time_point, msl_actuator_msgs::KickerStatInfoPtr>>
        kickerStatInfoMsgQueue;
    std::mutex kickerStatInfoMsgQueueMutex;
    std::queue<std::pair<std::chrono::system_clock::time_point, msl_sensor_msgs::SharedWorldInfoPtr>>
        sharedWorldInfoMsgQueue;
    std::mutex sharedWorldInfoMsgQueueMutex;

    void receiveKickerStatInfo(msl_actuator_msgs::KickerStatInfoPtr kickerStatInfo);
    void receiveSharedWorldInfo(msl_sensor_msgs::SharedWorldInfoPtr sharedWorldInfo);
    void processMessages();
    void checkAndInit(int robotId);

    QTimer *guiUpdateTimer;

  public Q_SLOTS:
    void run();
    void updateGUI();
    void showContextMenu(const QPoint &pos);
};
}
