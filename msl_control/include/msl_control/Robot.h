#pragma once

#include <chrono>
#include <process_manager/RobotMetaData.h>
#include <QFrame>
#include <ros/ros.h>
#include <alica_ros_proxy/AlicaEngineInfo.h>
#include <process_manager/ProcessStats.h>
#include <msl_actuator_msgs/KickerStatInfo.h>
#include <msl_sensor_msgs/SharedWorldInfo.h>

namespace Ui {
	class MSLRobotWidget;
}

namespace supplementary{
	class RobotExecutableRegistry;
}

namespace msl_control
{
	class MSLControl;

	class Robot : public QFrame, public supplementary::RobotMetaData
	{
		Q_OBJECT
	public:
		Robot(std::string robotName, int robotId, MSLControl* parentMSLControl);

		virtual ~Robot();

		// Message processing
		std::chrono::time_point<std::chrono::system_clock> timeLastMsgReceived; /**< the last time a message was received for this robot */
		void handleKickerStatInfo(std::pair<std::chrono::system_clock::time_point, msl_actuator_msgs::KickerStatInfoPtr> timeKSIpair);
		void handleSharedWorldInfo(std::pair<std::chrono::system_clock::time_point, msl_sensor_msgs::SharedWorldInfoPtr> timeSWIpair);

		// GUI Methods and Members
		MSLControl* parentMSLControl;
		void updateGUI(std::chrono::system_clock::time_point now);
		void toggle();
		void show();
		void hide();
		virtual QSize sizeHint();
		bool shown;

	private:
		QFrame* widget;
		Ui::MSLRobotWidget* uiMSLRobot;

//		QFrame* frameForPM;
//		pm_widget::ControlledRobot* controlledRobotWidget;

	};

} /* namespace msl_control */
