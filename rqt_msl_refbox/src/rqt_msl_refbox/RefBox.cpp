#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <rqt_msl_refbox/RefBox.h>

namespace rqt_msl_refbox {

RefBox::RefBox()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("RefBox");
}

void RefBox::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);
}

void RefBox::shutdownPlugin()
{
}

void RefBox::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{

}

void RefBox::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{

}

}

PLUGINLIB_EXPORT_CLASS(rqt_msl_refbox::RefBox, rqt_gui_cpp::Plugin)
