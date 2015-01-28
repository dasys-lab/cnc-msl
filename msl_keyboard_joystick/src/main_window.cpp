/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <math.h>
#include "../include/msl_keyboard_joystick/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace msl_keyboard_joystick {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    QObject::connect(ui.robotIdEdit, SIGNAL(textChanged(QString)), this, SLOT(onRobotIdEdited(QString)));

	/*********************
	** Logging
	**********************/
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    //QObject::connect(&qnode, SIGNAL(keyPressEvent()), this, SLOT(keyPressEvent(QKeyEvent *)));

    /*********************
    ** Auto Start
    **********************/
    /*if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }*/

    keyPressed[6];
    kick = false;
    robotId = 0;
    ui.robotIdEdit->setText(std::to_string(robotId).c_str());
    translation = 0;
    angle = 0;
    rotation = 0;
    kickPower = 0;
    shovelIdx = 0;
    ballHandleLeftMotor = 0;
    ballHandleRightMotor = 0;
    selectedActuator = 0;


}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */






/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
}

void MainWindow::keyPressEvent(QKeyEvent* event) {
    std::cout << event->key() << std::endl;

    switch (event->key()) {
    case Qt::Key_Up:
        keyPressed[0] = true;
        break;
    case Qt::Key_Down:
        keyPressed[1] = true;
        break;
    case Qt:: Key_Left:
        keyPressed[2] = true;
        break;
    case Qt::Key_Right:
        keyPressed[3] = true;
        break;
    case Qt::Key_Less:
        keyPressed[4] = true;
        break;
    case Qt::Key_Y:
        keyPressed[5] = true;
        break;
    case Qt::Key_Space:
        kick = true;
        break;
    case Qt::Key_Q:
        //rotation speed +
        if (rotation < 16 * M_PI) {
            rotation = rotation + M_PI / 8;
        }
        break;
    case Qt::Key_A:
        //rotation speed -
        if (rotation > 0) {
            rotation = rotation - M_PI / 8;
        }
        break;
    case Qt::Key_W:
        //kick power +
        if (kickPower < 3600) {
            kickPower = kickPower + 25;
        }
        break;
    case Qt::Key_S:
        //kick power -
        if (kickPower > 0) {
            kickPower = kickPower - 25;
        }
        break;
    case Qt::Key_E:

        //switch shovel

        if (shovelIdx == 0) {
            shovelIdx = 1;
        } else {
            shovelIdx = 0;
        }
        break;
    default:
        break;
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent* event) {
    std::cout << event->key() << std::endl;
    switch (event->key()) {
    case Qt::Key_Up:
        keyPressed[0] = false;
        break;
    case Qt::Key_Down:
        keyPressed[1] = false;
        break;
    case Qt:: Key_Left:
        keyPressed[2] = false;
        break;
    case Qt::Key_Right:
        keyPressed[3] = false;
        break;
    case Qt::Key_Less:
        keyPressed[4] = false;
        break;
    case Qt::Key_Y:
        keyPressed[5] = false;
        break;
    default:
        break;
    }
}

void MainWindow::onRobotIdEdited(QString text) {
    std::cout << text.toStdString() << std::endl;

    robotId = text.toInt();
}



/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace msl_keyboard_joystick

