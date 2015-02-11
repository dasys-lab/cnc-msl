#ifndef rqt_msl_refbox__RefBox_H
#define rqt_msl_refbox__RefBox_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_RefBox.h>

#include <ros/macros.h>

#include <QWidget>

namespace rqt_msl_refbox
{

	class RefBox : public rqt_gui_cpp::Plugin
	{

	Q_OBJECT

	public:

		RefBox();

		virtual void initPlugin(qt_gui_cpp::PluginContext& context);

		virtual void shutdownPlugin();

		virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

		virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

		Ui::RefBoxWidget ui_;

		QWidget* widget_;

	};

}

#endif // rqt_msl_refbox__RefBox_H
