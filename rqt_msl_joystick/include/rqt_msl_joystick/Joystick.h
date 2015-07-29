#ifndef rqt_msl_joystick__Joystick_H
#define rqt_msl_joystick__Joystick_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_Joystick.h>

#include "ros/ros.h"
#include <ros/macros.h>

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

		virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);


		Ui::JoystickWidget uiJoystickWidget;

		QWidget* uiWidget;

	private:

		bool eventFilter(QObject* watched, QEvent* event);

	};

}

#endif // rqt_msl_refbox__RefBox_H
