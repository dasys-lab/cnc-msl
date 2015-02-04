/**
 * @file /include/msl_refbox/main_window.hpp
 *
 * @brief Qt based gui for msl_refbox.
 *
 * @date November 2010
 **/
#ifndef msl_refbox_MAIN_WINDOW_H
#define msl_refbox_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace msl_refbox {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class RefoboxWindow : public QMainWindow {
Q_OBJECT

public:
	RefoboxWindow(int argc, char** argv, QWidget *parent = 0);
	~RefoboxWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::RefboxWindowDesign ui;
	QNode qnode;
};

}  // namespace msl_refbox

#endif // msl_refbox_MAIN_WINDOW_H
