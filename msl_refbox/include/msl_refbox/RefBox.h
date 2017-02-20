#ifndef msl_refbox__RefBox_H
#define msl_refbox__RefBox_H

#include <msl_refbox/GameData.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_RefBox.h>

#include "ros/ros.h"
#include <ros/macros.h>

#include <QDialog>
#include <QWidget>
#include <QtGui>

namespace msl_refbox
{
using namespace std;

class GameData;
class RefBox : public rqt_gui_cpp::Plugin, public Ui::RefBoxWidget
{

    Q_OBJECT

  public:
    RefBox();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);

    void debugLog(std::string msg);

    Ui::RefBoxWidget ui_;
    QWidget *widget_;

  public Q_SLOTS:
    void onDebugToggled(bool);

  private:
    GameData *gameData;
    bool debug;
    //		bool eventFilter(QObject* watched, QEvent* event);
};
}

#endif // msl_refbox__RefBox_H
