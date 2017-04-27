#include <memory.h>
#include <msl_refbox/GameData.h>
#include <msl_refbox/RefBox.h>
#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <QStyleFactory>

namespace msl_refbox
{

RefBox::RefBox()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    setObjectName("RefBox");
    gameData = new GameData(this);
    this->debug = false;
}

void RefBox::initPlugin(qt_gui_cpp::PluginContext &context)
{
    // used to enable colored buttons
    QApplication::setStyle(QStyleFactory::create("fusion"));
    widget_ = new QWidget();
    setupUi(widget_);
    if (context.serialNumber() > 1)
    {
        widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(widget_);

    //		this->btn_connect->setEnabled(false);
    connect(Play_On_bot, SIGNAL(clicked()), gameData, SLOT(PlayOnPressed()));
    connect(Stop_bot, SIGNAL(clicked()), gameData, SLOT(StopPressed()));
    connect(Halt_bot, SIGNAL(clicked()), gameData, SLOT(HaltPressed()));
    connect(Dropped_bot, SIGNAL(clicked()), gameData, SLOT(DroppedBallPressed()));
    connect(Parking_bot, SIGNAL(clicked()), gameData, SLOT(ParkingPressed()));

    // our
    connect(Our_Kick_Off_bot, SIGNAL(clicked()), gameData, SLOT(OurKickOffPressed()));
    connect(Our_Free_Kick_bot, SIGNAL(clicked()), gameData, SLOT(OurFreeKickPressed()));
    connect(Our_Goal_Kick_bot, SIGNAL(clicked()), gameData, SLOT(OurGoalKickPressed()));
    connect(Our_Throwin_bot, SIGNAL(clicked()), gameData, SLOT(OurThrowinPressed()));
    connect(Our_Corner_Kick_bot, SIGNAL(clicked()), gameData, SLOT(OurCornerKickPressed()));
    connect(Our_Penalty_bot, SIGNAL(clicked()), gameData, SLOT(OurPenaltyPressed()));

    // their
    connect(Their_Kick_Off_bot, SIGNAL(clicked()), gameData, SLOT(TheirKickOffPressed()));
    connect(Their_Free_Kick_bot, SIGNAL(clicked()), gameData, SLOT(TheirFreeKickPressed()));
    connect(Their_Goal_Kick_bot, SIGNAL(clicked()), gameData, SLOT(TheirGoalKickPressed()));
    connect(Their_Throwin_bot, SIGNAL(clicked()), gameData, SLOT(TheirThrowinPressed()));
    connect(Their_Corner_Kick_bot, SIGNAL(clicked()), gameData, SLOT(TheirCornerKickPressed()));
    connect(Their_Penalty_bot, SIGNAL(clicked()), gameData, SLOT(TheirPenaltyPressed()));

    connect(Joystick_bot, SIGNAL(clicked()), gameData, SLOT(JoystickPressed()));

    // debug
    connect(chk_debug, SIGNAL(toggled(bool)), this, SLOT(onDebugToggled(bool)));

    // Connect Information
    connect(rbtn_local, SIGNAL(toggled(bool)), gameData, SLOT(onLocalToggled(bool)));

    connect(rbtn_xml, SIGNAL(toggled(bool)), gameData, SLOT(onXmlToggled(bool)));
    connect(rbtn_char, SIGNAL(toggled(bool)), gameData, SLOT(onCharToggled(bool)));

    connect(rbtn_tcp, SIGNAL(toggled(bool)), gameData, SLOT(onTcpToggled(bool)));
    connect(rbtn_udp, SIGNAL(toggled(bool)), gameData, SLOT(onUdpToggled(bool)));
    connect(chk_reconnect, SIGNAL(toggled(bool)), gameData, SLOT(onReconnectToggled(bool)));

    connect(btn_connect, SIGNAL(clicked()), gameData, SLOT(onConnectPressed()));

    widget_->installEventFilter(this);
    this->tbl_info->setRowCount(15);
    this->tbl_info->setColumnCount(4);
    this->tbl_info->verticalHeader()->setVisible(false);
    this->tbl_info->horizontalHeader()->setVisible(false);
    this->tbl_info->verticalHeader()->setDefaultSectionSize(16);
    //this->tbl_info->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
    this->tbl_info->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);


    QTableWidgetItem *item = new QTableWidgetItem();
    item->setText("Team");
    this->tbl_info->setItem(0, 0, item);

    QTableWidgetItem *item1 = new QTableWidgetItem();
    item1->setText("Player");
    this->tbl_info->setItem(0, 1, item1);

    QTableWidgetItem *item2 = new QTableWidgetItem();
    item2->setText("In Field");
    this->tbl_info->setItem(0, 2, item2);

    QTableWidgetItem *item3 = new QTableWidgetItem();
    item3->setText("Card");
    this->tbl_info->setItem(0, 3, item3);
}

//	bool RefBox::eventFilter(QObject* watched, QEvent* event)
//	{
//		if(!gameData->localToggled
//				&& (gameData->udpToggled || gameData->tcpToggled)
//				&& ledit_ipaddress->text().size() > 5
//				&& ledit_ipaddress->text().count(".") == 3
//				&& spin_port->value() > 0)
//		{
//			btn_connect->setEnabled(true);
//		}
//		else
//			btn_connect->setEnabled(false);
//		return true;
//	}

void RefBox::shutdownPlugin()
{
    disconnect(chk_debug, SIGNAL(toggled(bool)), this, SLOT(onDebugToggled(bool)));

    disconnect(Play_On_bot, SIGNAL(clicked()), gameData, SLOT(PlayOnPressed()));
    disconnect(Stop_bot, SIGNAL(clicked()), gameData, SLOT(StopPressed()));
    disconnect(Halt_bot, SIGNAL(clicked()), gameData, SLOT(HaltPressed()));
    disconnect(Dropped_bot, SIGNAL(clicked()), gameData, SLOT(DroppedBallPressed()));
    disconnect(Parking_bot, SIGNAL(clicked()), gameData, SLOT(ParkingPressed()));

    disconnect(Our_Kick_Off_bot, SIGNAL(clicked()), gameData, SLOT(OurKickOffPressed()));
    disconnect(Our_Free_Kick_bot, SIGNAL(clicked()), gameData, SLOT(OurFreeKickPressed()));
    disconnect(Our_Goal_Kick_bot, SIGNAL(clicked()), gameData, SLOT(OurGoalKickPressed()));
    disconnect(Our_Throwin_bot, SIGNAL(clicked()), gameData, SLOT(OurThrowinPressed()));
    disconnect(Our_Corner_Kick_bot, SIGNAL(clicked()), gameData, SLOT(OurCornerKickPressed()));
    disconnect(Our_Penalty_bot, SIGNAL(clicked()), gameData, SLOT(OurPenaltyPressed()));

    disconnect(Their_Kick_Off_bot, SIGNAL(clicked()), gameData, SLOT(TheirKickOffPressed()));
    disconnect(Their_Free_Kick_bot, SIGNAL(clicked()), gameData, SLOT(TheirFreeKickPressed()));
    disconnect(Their_Goal_Kick_bot, SIGNAL(clicked()), gameData, SLOT(TheirGoalKickPressed()));
    disconnect(Their_Throwin_bot, SIGNAL(clicked()), gameData, SLOT(TheirThrowinPressed()));
    disconnect(Their_Corner_Kick_bot, SIGNAL(clicked()), gameData, SLOT(TheirCornerKickPressed()));
    disconnect(Their_Penalty_bot, SIGNAL(clicked()), gameData, SLOT(TheirPenaltyPressed()));

    disconnect(Joystick_bot, SIGNAL(clicked()), gameData, SLOT(JoystickPressed()));
    disconnect(rbtn_local, SIGNAL(toggled(bool)), gameData, SLOT(onLocalToggled(bool)));
    disconnect(rbtn_xml, SIGNAL(toggled(bool)), gameData, SLOT(onXmlToggled(bool)));
    disconnect(rbtn_char, SIGNAL(toggled(bool)), gameData, SLOT(onCharToggled(bool)));
    disconnect(rbtn_tcp, SIGNAL(toggled(bool)), gameData, SLOT(onTcpToggled(bool)));
    disconnect(rbtn_udp, SIGNAL(toggled(bool)), gameData, SLOT(onUdpToggled(bool)));
    disconnect(btn_connect, SIGNAL(clicked()), gameData, SLOT(onConnectPressed()));

    delete gameData;
}

void RefBox::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
{
}

void RefBox::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
{
}

void RefBox::onDebugToggled(bool checked)
{
    this->debug = checked;

    if (checked)
        std::cout << "Debug mode enabled" << std::endl;
    else
        std::cout << "Debug mode disabled" << std::endl;
}

void RefBox::debugLog(std::string msg)
{
    if (false == this->debug)
        return;

    std::cout << msg << std::endl;
}
}

PLUGINLIB_EXPORT_CLASS(msl_refbox::RefBox, rqt_gui_cpp::Plugin)
