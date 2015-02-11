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


#include "ros/ros.h"
#include "msl_msgs/JoystickCommand.h"

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
    void onRobotIdEdited(QString text);
    void updateLoggingView(); // no idea why this can't connect automatically

private:   

    void sendJoystickMessage();

    ros::Publisher joystickpub;
    ros::NodeHandle n;
    ros::AsyncSpinner* spinner;
    int robotId;
    double angle, translation, rotation;
    unsigned short kickPower;
    char shovelIdx, ballHandleLeftMotor, ballHandleRightMotor, selectedActuator;
    bool *keyPressed;
    bool kick;
	void keyPressEvent(QKeyEvent* event); 
    void keyReleaseEvent(QKeyEvent* event);
    bool checkNumber(QString text);
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace msl_keyboard_joystick

#endif // msl_keyboard_joystick_MAIN_WINDOW_H
