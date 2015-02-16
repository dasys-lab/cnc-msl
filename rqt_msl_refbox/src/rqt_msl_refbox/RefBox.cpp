#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <rqt_msl_refbox/RefBox.h>

#include "rqt_msl_refbox/RefBoxCommunication.h"

namespace rqt_msl_refbox
{

	RefBox::RefBox() :
			rqt_gui_cpp::Plugin(), widget_(0)
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

		widget_->installEventFilter(this);

		this->RBCommQDialog = new QDialog(widget_);
		this->RBComm = new RefBoxCommunication(this->RBCommQDialog);

	}

	bool RefBox::eventFilter(QObject* watched, QEvent* event)
	{
		if (watched == widget_ && event->type() == QEvent::KeyPress)
		{
			QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

			if (keyEvent->key() == Qt::Key_R)
			{
				cout << "R pressed" << endl;
				this->showRBDialog();
				return true;
			}
		}
		return true;
	}

	void RefBox::showRBDialog()
	{
		RBComm->update_manual_config();
		RBCommQDialog->show();
	}

	void RefBox::shutdownPlugin()
	{
		delete RBComm;
		delete RBCommQDialog;
	}

	void RefBox::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
	{

	}

	void RefBox::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
	{

	}

}

PLUGINLIB_EXPORT_CLASS(rqt_msl_refbox::RefBox, rqt_gui_cpp::Plugin)
