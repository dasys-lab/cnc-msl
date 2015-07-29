#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <rqt_msl_joystick/Joystick.h>

using namespace std;

namespace rqt_msl_joystick
{

	using namespace Qt;

	void Joystick::sendJoystickMessage()
	{

		msl_msgs::JoystickCommand msg;
		msg.ballHandleLeftMotor = ballHandleLeftMotor;
		msg.ballHandleRightMotor = ballHandleRightMotor;
		msg.kick = kick;
		msg.kickPower = kickPower;
		msg.robotId = robotId;
		msg.selectedActuator = selectedActuator;
		msg.shovelIdx = shovelIdx;
		msg.motion.translation = translation;
		msg.motion.rotation = rotation;

		double motionAngle = 0;

		// add motion
		// 0 == up
		// 1 == down
		// 2 == left
		// 3 == right

		if (keyPressed[0] == true && keyPressed[2] == true)
		{

			motionAngle = (0.75 * M_PI);

		}
		else if (keyPressed[0] == true && keyPressed[3] == true)
		{

			motionAngle = (1.25 * M_PI);

		}
		else if (keyPressed[1] == true && keyPressed[2] == true)
		{

			motionAngle = (0.25 * M_PI);

		}
		else if (keyPressed[1] == true && keyPressed[3] == true)
		{

			motionAngle = (1.75 * M_PI);

		}
		else if (keyPressed[0] == true)
		{

			motionAngle = M_PI;

		}
		else if (keyPressed[1] == true)
		{

			motionAngle = 0;

		}
		else if (keyPressed[2] == true)
		{

			motionAngle = (1.5 * M_PI);

		}
		else if (keyPressed[3] == true)
		{

			motionAngle = (0.5 * M_PI);

		}
		else
		{

			motionAngle = 0;
			msg.motion.translation = 0;
			msg.motion.rotation = 0;

		}

		ui.motionValue->setText(std::to_string(motionAngle).c_str());
		ui.kickPowerValue->setText(std::to_string(kickPower).c_str());
		ui.translationValue->setText(std::to_string(translation).c_str());
		ui.shovelValue->setText(std::to_string(shovelIdx).c_str());

		msg.motion.angle = motionAngle;

		if (keyPressed[4] == true)
		{

			msg.motion.rotation = rotation;

		}
		else if (keyPressed[5] == true)
		{

			msg.motion.rotation = -rotation;

		}
		else
		{

			msg.motion.rotation = 0;

		}
		ui.rotationValue->setText(std::to_string(rotation).c_str());
		joystickpub.publish(msg);
	}

	void Joystick::keyPressEvent(QKeyEvent* event)
	{

		if (!(event->isAutoRepeat()))
		{
			std::cout << "P: " << event->key() << std::endl;

			switch (event->key())
			{
				case Qt::Key_Up:
					keyPressed[0] = true;
					break;
				case Qt::Key_Down:
					keyPressed[1] = true;
					break;
				case Qt::Key_Left:
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
					if (rotation < 16 * M_PI)
					{
						rotation = rotation + M_PI / 8;
					}
					break;
				case Qt::Key_A:
					//rotation speed -
					if (rotation > 0)
					{
						rotation = rotation - M_PI / 8;
					}
					break;
				case Qt::Key_W:
					//kick power +
					if (kickPower < 3600)
					{
						kickPower = kickPower + 25;
					}
					break;
				case Qt::Key_S:
					//kick power -
					if (kickPower > 0)
					{
						kickPower = kickPower - 25;
					}
					break;
				case Qt::Key_R:
					//Speed (translation) +
					//TODO: After test for actuate pls turn down to 1000
					if (translation < 4000)
					{
						translation = translation + 100;
					}
					break;
				case Qt::Key_F:
					//Speed (translation) -
					if (translation > 0)
					{
						translation = translation - 100;
					}
					break;
				case Qt::Key_E:

					//switch shovel

					if (shovelIdx == 0)
					{
						shovelIdx = 1;
					}
					else
					{
						shovelIdx = 0;
					}
					break;
				default:
					break;
			}
		}
		sendJoystickMessage();

	}

	void Joystick::keyReleaseEvent(QKeyEvent* event)
	{

		if (!(event->isAutoRepeat()))
		{
			std::cout << "R: " << event->key() << std::endl;

			switch (event->key())
			{
				case Qt::Key_Up:
					keyPressed[0] = false;
					break;
				case Qt::Key_Down:
					keyPressed[1] = false;
					break;
				case Qt::Key_Left:
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
				case Qt::Key_Space:
					kick = false;
					break;
				default:
					break;
			}
		}
		sendJoystickMessage();

	}

	void Joystick::onRobotIdEdited()
	{
		this->robotId = ui.robotIdEdit->text().toInt();
		ui.robotIdEdit->clearFocus();
		cout << "We got the Enter-Event!" << endl;

	}

	bool Joystick::checkNumber(QString text)
	{
		// check if String contains a letter

		int textSize = text.size();

		//convert QString to char*
		char* cText = text.toLatin1().data();

		for (int i = 0; i < textSize; i++)
		{
			if (isalpha(cText[i]))
			{
				//delete cText;
				return false;
			}
		}
		//delete cText;
		return true;

	}

	Joystick::Joystick() : rqt_gui_cpp::Plugin(), widget(0)
	{
		setObjectName("Joystick");
		ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
		QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

		setWindowIcon(QIcon(":/images/icon.png"));
		QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

		QObject::connect(ui.robotIdEdit, SIGNAL(returnPressed()), this, SLOT(onRobotIdEdited()));

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

		joystickpub = n.advertise < msl_msgs::JoystickCommand > ("/Joystick", 1);
		spinner = new ros::AsyncSpinner(2);
		spinner->start();
		keyPressed = vector<bool>(6);
		for (int i = 0; i < keyPressed.size(); i++)
		{
			keyPressed[i] = false;
		}
		kick = false;
		robotId = 0;
		//		ui.robotIdEdit->setText(std::to_string(robotId).c_str());
		translation = 0;
		angle = 0;
		rotation = 0;
		kickPower = 0;
		shovelIdx = 0;
		ballHandleLeftMotor = 0;
		ballHandleRightMotor = 0;
		selectedActuator = 0;
		//ui.robotIdEdit->setVisible(false);
	}

	void Joystick::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		//used to enable colored buttons
		QApplication::setStyle(new QPlastiqueStyle);
		widget_ = new QWidget();
		setupUi (widget_);
		if (context.serialNumber() > 1)
		{
			widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
		}
		context.addWidget(widget_);

	}

	void Joystick::shutdownPlugin()
	{
		// TODO
	}

	void Joystick::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
	{

	}

	void Joystick::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
									const qt_gui_cpp::Settings& instance_settings)
	{

	}

} // namespace msl_keyboard_joystick

PLUGINLIB_EXPORT_CLASS(rqt_msl_joystick::Joystick, rqt_gui_cpp::Plugin)

