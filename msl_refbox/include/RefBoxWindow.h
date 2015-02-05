#ifndef msl_refbox_REFBOXWINDOW_H
#define msl_refbox_REFBOXWINDOW_H

#include <QtGui/QMainWindow>
#include "ui_RefBoxWindow.h"

namespace msl_refbox
{
	class RefBoxWindow : public QMainWindow, public Ui::RefBoxWindowDesign
	{
	Q_OBJECT

	public:
		RefBoxWindow(QMainWindow *parent = 0);
		~RefBoxWindow();
		void closeEvent(QCloseEvent *event); // Overloaded function

	protected:
		bool eventFilter(QObject *obj, QEvent *event);

	private:
		Ui::RefBoxWindowDesign ui;

	};

} // namespace msl_refbox

#endif // msl_refbox_REFBOXWINDOW_H
