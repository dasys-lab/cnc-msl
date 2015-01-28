/**
 * @file /include/msl_keyboard_joystick/main_window.hpp
 *
 * @brief Qt based gui for msl_keyboard_joystick.
 *
 * @date November 2010
 **/
#ifndef msl_keyboard_joystick_MAIN_WINDOW_H
#define msl_keyboard_joystick_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace msl_keyboard_joystick {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/


    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
    void keyPressEvent(QKeyEvent* event);
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace msl_keyboard_joystick

#endif // msl_keyboard_joystick_MAIN_WINDOW_H
