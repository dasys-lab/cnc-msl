#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <msl_control/MSLControl.h>

#include <SystemConfig.h>
#include <process_manager/RobotExecutableRegistry.h>

#include <QMenu>

namespace msl_control
{

std::chrono::duration<double> MSLControl::msgTimeOut = std::chrono::duration<double>(0);

MSLControl::MSLControl()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
    , guiUpdateTimer(nullptr)
{
    setObjectName("MSLControl");
    rosNode = new ros::NodeHandle();

    this->sc = supplementary::SystemConfig::getInstance();
    MSLControl::msgTimeOut = std::chrono::duration<double>(
        (*this->sc)["ProcessManaging"]->get<unsigned long>("PMControl.timeLastMsgReceivedTimeOut", NULL));
    this->pmRegistry = supplementary::RobotExecutableRegistry::get();

    /* Initialise the registry data structure for better performance
     * with data from Globals.conf and Processes.conf file. */

    // Register robots from Globals.conf
    auto robotNames = (*this->sc)["Globals"]->getSections("Globals.Team", NULL);
    for (auto robotName : (*robotNames))
    {
        this->pmRegistry->addRobot(robotName);
    }

    // TODO: needs to be expanded for arbitrary new robots (when discovery module is ready)
}

void MSLControl::initPlugin(qt_gui_cpp::PluginContext &context)
{
    widget_ = new QWidget();
    widget_->setAttribute(Qt::WA_AlwaysShowToolTips, true);
    robotControlWidget_.setupUi(widget_);

    this->widget_->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);
    QObject::connect(this->widget_, SIGNAL(customContextMenuRequested(const QPoint &)), this,
                     SLOT(showContextMenu(const QPoint &)));

    if (context.serialNumber() > 1)
    {
        widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(widget_);

    for (auto robot : this->pmRegistry->getRobots())
    {
        this->checkAndInit(robot->id);
    }

    // Initialise the ROS Communication
    kickerStatInfoSub =
        rosNode->subscribe("/KickerStatInfo", 10, &MSLControl::receiveKickerStatInfo, (MSLControl *)this);
    sharedWorldInfoSub =
        rosNode->subscribe("/WorldModel/SharedWorldInfo", 10, &MSLControl::receiveSharedWorldInfo, (MSLControl *)this);

    // Initialise the GUI refresh timer
    this->guiUpdateTimer = new QTimer();
    QObject::connect(guiUpdateTimer, SIGNAL(timeout()), this, SLOT(run()));
    this->guiUpdateTimer->start(200);
}

void MSLControl::showContextMenu(const QPoint &pos)
{
    /* HINT: remember, if there are some problems that way:
     * For QAbstractScrollArea and derived classes you would use:
     * QPoint globalPos = myWidget->viewport()->mapToGlobal(pos); */

    QPoint globalPos = this->widget_->mapToGlobal(pos);

    QMenu myMenu;
    for (auto robot : this->pmRegistry->getRobots())
    {
        myMenu.addAction(std::string(robot->name + " (" + std::to_string(robot->id) + ")").c_str());
    }

    QAction *selectedItem = myMenu.exec(globalPos);
    if (selectedItem)
    {
        int robotId;

        std::string name = selectedItem->iconText().toStdString().substr();
        name = name.substr(0, name.find('(') - 1);

        cout << "RC: '" << name << "'" << endl;

        if (this->pmRegistry->getRobotId(name, robotId))
        {
            // this->checkAndInit(robotId);
            this->controlledRobotsMap[robotId]->toggle();
        }
        else
        {
            cerr << "RC: Chosen robot is not known in the robot registry!" << endl;
        }
    }
    else
    {
        cout << "RC: Nothing chosen!" << endl;
    }
}

/**
 * The worker method of MSLControl. It processes the received ROS messages and afterwards updates the GUI.
 */
void MSLControl::run()
{
    processMessages();

    updateGUI();
}

/**
 * Updates the GUI, after ROS process stat message have been processed.
 */
void MSLControl::updateGUI()
{
    chrono::system_clock::time_point now = chrono::system_clock::now();
    for (auto controlledRobotEntry : this->controlledRobotsMap)
    {
        controlledRobotEntry.second->updateGUI(now);
    }
}

void MSLControl::receiveKickerStatInfo(msl_actuator_msgs::KickerStatInfoPtr kickerStatInfo)
{
    lock_guard<mutex> lck(kickerStatInfoMsgQueueMutex);
    this->kickerStatInfoMsgQueue.emplace(chrono::system_clock::now(), kickerStatInfo);
}

void MSLControl::receiveSharedWorldInfo(msl_sensor_msgs::SharedWorldInfoPtr sharedWorldInfo)
{
    lock_guard<mutex> lck(sharedWorldInfoMsgQueueMutex);
    this->sharedWorldInfoMsgQueue.emplace(chrono::system_clock::now(), sharedWorldInfo);
}

/**
 * Processes all queued messages from the processStatMsgsQueue and the alicaInfoMsgQueue.
 */
void MSLControl::processMessages()
{
    {
        lock_guard<mutex> lck(kickerStatInfoMsgQueueMutex);
        while (!this->kickerStatInfoMsgQueue.empty())
        {
            // unqueue the ROS kicker stat info message
            auto timeKickerStatInfoPair = kickerStatInfoMsgQueue.front();
            kickerStatInfoMsgQueue.pop();

            this->controlledRobotsMap[timeKickerStatInfoPair.second->senderID]->handleKickerStatInfo(
                timeKickerStatInfoPair);
        }
    }

    {
        lock_guard<mutex> lck(sharedWorldInfoMsgQueueMutex);
        while (!this->sharedWorldInfoMsgQueue.empty())
        {
            // unqueue the ROS shared world info message
            auto timeSharedWorldInfoPair = sharedWorldInfoMsgQueue.front();
            sharedWorldInfoMsgQueue.pop();

            this->controlledRobotsMap[timeSharedWorldInfoPair.second->senderID]->handleSharedWorldInfo(
                timeSharedWorldInfoPair);
        }
    }
}

/**
 * If the given robot ID is already known, nothing is done.
 * Otherwise a new entry in the controlled robot map is created.
 */
void MSLControl::checkAndInit(int robotId)
{
    auto pmEntry = this->controlledRobotsMap.find(robotId);
    if (pmEntry == this->controlledRobotsMap.end())
    { // robot is not known, so create a corresponding instance
        string robotName;
        if (this->pmRegistry->getRobotName(robotId, robotName))
        {
            cout << "RC: Create new ControlledRobot with ID " << robotId << " and host name " << robotName << "!"
                 << endl;
            Robot *controlledRobot = new Robot(robotName, robotId, this);
            this->controlledRobotsMap.emplace(robotId, controlledRobot);
        }
        else
        {
            cerr << "RC: Received message from unknown robot with sender id " << robotId << endl;
        }
    }
}

void MSLControl::shutdownPlugin()
{
    this->kickerStatInfoSub.shutdown();
    this->sharedWorldInfoSub.shutdown();
}

void MSLControl::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
{
}

void MSLControl::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings)
{
}
}

PLUGINLIB_EXPORT_CLASS(msl_control::MSLControl, rqt_gui_cpp::Plugin)
