#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <rqt_msl_refbox/GameData.h>

#include <rqt_msl_refbox/RefBox.h>

namespace rqt_msl_refbox
{

	RefBox::RefBox() :
			rqt_gui_cpp::Plugin(), widget_(0)
	{
		setObjectName("RefBox");
		gameData = new GameData(this);
	}

	void RefBox::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		//used to enable colored buttons
		QApplication::setStyle(new QPlastiqueStyle);
		widget_ = new QWidget();
		setupUi(widget_);
		if (context.serialNumber() > 1)
		{
			widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
		}
		context.addWidget(widget_);

		this->btn_connect->setEnabled(false);
		connect(Play_On_bot, SIGNAL(clicked()), gameData, SLOT(PlayOnPressed()));
		connect(Stop_bot, SIGNAL(clicked()), gameData, SLOT(StopPressed()));
		connect(Halt_bot, SIGNAL(clicked()), gameData, SLOT(HaltPressed()));
		connect(Dropped_bot, SIGNAL(clicked()), gameData, SLOT(DroppedBallPressed()));
		connect(Parking_bot, SIGNAL(clicked()), gameData, SLOT(ParkingPressed()));

		//our
		connect(Our_Kick_Off_bot, SIGNAL(clicked()), gameData, SLOT(OurKickOffPressed()));
		connect(Our_Free_Kick_bot, SIGNAL(clicked()), gameData, SLOT(OurFreeKickPressed()));
		connect(Our_Goal_Kick_bot, SIGNAL(clicked()), gameData, SLOT(OurGoalKickPressed()));
		connect(Our_Throwin_bot, SIGNAL(clicked()), gameData, SLOT(OurThrowinPressed()));
		connect(Our_Corner_Kick_bot, SIGNAL(clicked()), gameData, SLOT(OurCornerKickPressed()));
		connect(Our_Penalty_bot, SIGNAL(clicked()), gameData, SLOT(OurPenaltyPressed()));

		//their
		connect(Their_Kick_Off_bot, SIGNAL(clicked()), gameData, SLOT(TheirKickOffPressed()));
		connect(Their_Free_Kick_bot, SIGNAL(clicked()), gameData, SLOT(TheirFreeKickPressed()));
		connect(Their_Goal_Kick_bot, SIGNAL(clicked()), gameData, SLOT(TheirGoalKickPressed()));
		connect(Their_Throwin_bot, SIGNAL(clicked()), gameData, SLOT(TheirThrowinPressed()));
		connect(Their_Corner_Kick_bot, SIGNAL(clicked()), gameData, SLOT(TheirCornerKickPressed()));
		connect(Their_Penalty_bot, SIGNAL(clicked()), gameData, SLOT(TheirPenaltyPressed()));

		connect(Joystick_bot, SIGNAL(clicked()), gameData, SLOT(JoystickPressed()));

		//Connectinformation
		connect(rbtn_local, SIGNAL(toggled(bool)), gameData, SLOT(onLocalTogled(bool)));
		connect(rbtn_xmlMulti, SIGNAL(toggled(bool)), gameData, SLOT(onMultiTogled(bool)));
		connect(rbtn_xmlTcp, SIGNAL(toggled(bool)), gameData, SLOT(onTcpTogled(bool)));

		connect(btn_connect, SIGNAL(clicked()), gameData, SLOT(onConnectPressed()));

		widget_->installEventFilter(this);
//
//		this->refBoxCommQDialog = new QDialog(widget_);
//		this->_refBoxCommQDialog = new RefBoxCommunication(this->refBoxCommQDialog);

	}

	//TODO CLEAN UP
	bool RefBox::eventFilter(QObject* watched, QEvent* event)
	{
		if(!gameData->isLocalToggled()
				&& (gameData->isMultiToggled() || gameData->isTcpToggled())
				&& ledit_ipaddress->text().size() > 5
				&& ledit_ipaddress->text().count(".") == 3
				&& spin_port->value() > 0)
		{
			btn_connect->setEnabled(true);
		}
		else
			btn_connect->setEnabled(false);
		return true;
	}

//	void RefBox::showRBDialog()
//	{
////		refBoxCommQDialog->show();
//	}

	void RefBox::shutdownPlugin()
	{
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
		disconnect(rbtn_local, SIGNAL(toggled(bool)), gameData, SLOT(onLocalTogled(bool)));
		disconnect(rbtn_xmlMulti, SIGNAL(toggled(bool)), gameData, SLOT(onMultiTogled(bool)));
		disconnect(rbtn_xmlTcp, SIGNAL(toggled(bool)), gameData, SLOT(onTcpTogled(bool)));

				connect(btn_connect, SIGNAL(clicked()), gameData, SLOT(onConnectPressed()));
		delete gameData;
	}

	void RefBox::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
	{

	}


	void RefBox::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
									const qt_gui_cpp::Settings& instance_settings)
	{

	}

}

PLUGINLIB_EXPORT_CLASS(rqt_msl_refbox::RefBox, rqt_gui_cpp::Plugin)
