#ifndef msl_joystick__Joystick_H
#define msl_joystick__Joystick_H
#define RQT_MSL_JOYSTICK_DEBUG

#include <rqt_gui_cpp/plugin.h>
#include <ui_Joystick.h>

#include "ros/ros.h"
#include <ros/macros.h>
#include <msl_msgs/JoystickCommand.h>

#include <stdint.h>
#include <vector>
#include <QtGui>
#include <QWidget>
#include <QDialog>



namespace msl_joystick
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


		bool eventFilter(QObject* watched, QEvent* event);
		void keyPressEvent(QKeyEvent* event);
		void keyReleaseEvent(QKeyEvent* event);
		void printControlValues();
		void printJoystickMessage(msl_msgs::JoystickCommand msg);

		QWidget* uiWidget;

	public Q_SLOTS:
		void sendJoystickMessage();

		void onRobotIdEdited();
		void onKickPowerEdited();
		void onRotationEdited();
		void onTranslationEdited();
		void onBallHandleRightEdited();
		void onBallHandleLeftEdited();

		void onKickPowerSlided(int value);
		void onBallHandleRightSlided(int value);
		void onBallHandleLeftSlided(int value);

		void onLowShovelSelected(bool checked);
		void onHighShovelSelected(bool checked);
		void onBallHandleCheckBoxToggled(int checkState);

	private:

		ros::NodeHandle* rosNode;
		ros::Publisher joyPub;
		ros::AsyncSpinner* spinner;

		vector<bool> keyPressed;

		// for filling a joystick message
		int ballHandleLeftMotor;
		int ballHandleRightMotor;
		short kickPower;
		int robotId;
		bool useBallHandle;
		short shovelIdx;
		double translation;
		double rotation;

		// min max values from config
		int ballHandleMin;
		int ballHandleMax;
		short kickPowerMin;
		short kickPowerMax;
		double translationMin;
		double translationMax;
		double rotationMin;
		double rotationMax;

		QTimer* sendMsgTimer;
		int sendInterval;
	};

}

#endif // msl_joystick__Joystick_H
