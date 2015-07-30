#ifndef rqt_msl_joystick__Joystick_H
#define rqt_msl_joystick__Joystick_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_Joystick.h>

#include "ros/ros.h"
#include <ros/macros.h>

#include <stdint.h>
#include <vector>
#include <QtGui>
#include <QWidget>
#include <QDialog>

namespace rqt_msl_joystick
{
	using namespace std;

	class Joystick : public rqt_gui_cpp::Plugin, public Ui::JoystickWidget
	{

	Q_OBJECT

	public:

		Joystick();

		virtual void initPlugin(qt_gui_cpp::PluginContext& context);

		virtual void shutdownPlugin();

		virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

		virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
										const qt_gui_cpp::Settings& instance_settings);

		void sendJoystickMessage();
		void keyPressEvent(QKeyEvent* event);
		void keyReleaseEvent(QKeyEvent* event);
		bool checkNumber(QString text);
		void printControlValues();

		QWidget* uiWidget;

	public Q_SLOTS:
		void onRobotIdEdited();
		void onKickPowerEdited();
		void onRotationEdited();
		void onTranslationEdited();

		void onKickPowerSlided();
		void onTranslationSlided();
		void onRotationSpeedSlided();
		void onBallHandleRightSlided();
		void onBallHandleRightEdited();
		void onBallHandleLeftSlided();
		void onBallHandleLeftEdited();

		void onLowShovelSelected(bool checked);
		void onHighShovelSelected(bool checked);

	private:

		ros::NodeHandle* rosNode;
		ros::Publisher joyPub;
		ros::AsyncSpinner* spinner;

		vector<bool> keyPressed;

		// for filling a joystick message
		short ballHandleLeftMotor;
		short ballHandleRightMotor;
		bool kick;
		short kickPower;
		int robotId;
		short selectedActuator;
		short shovelIdx;
		double angle;
		double translation;
		double rotation;

	};

}

#endif // rqt_msl_joystick__Joystick_H
