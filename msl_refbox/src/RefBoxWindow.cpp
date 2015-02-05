#include <RefBoxWindow.h>
#include <QtGui>
#include <QMessageBox>
#include <iostream>

namespace msl_refbox
{

	using namespace Qt;
	using namespace std;

	RefBoxWindow::RefBoxWindow(QMainWindow *parent) :
			QMainWindow(parent)
	{
		ui.setupUi(this);

		setWindowIcon(QIcon(":/images/cn_bat_only.png"));

		/* qApp is a global static variable for the whole
		 * application, therefore, all events will be passed
		 * through the eventFilter method of this class */
		qApp->installEventFilter(this);

	}

	RefBoxWindow::~RefBoxWindow()
	{
	}

	bool RefBoxWindow::eventFilter(QObject *obj, QEvent *event)
	{

		if (obj == this)
		{
			if (event->type() == QEvent::KeyPress)
			{

				QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

				if (keyEvent->key() == Qt::Key_C)
				{
					RefBoxWidget->detailsBotPressed();
					return true;
				}
			}
		}
		return false;
	}


	void RefBoxWindow::closeEvent(QCloseEvent *event)
	{
		QMainWindow::closeEvent(event);
	}

} // namespace msl_refbox

