#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <rqt_msl_joystick/Joystick.h>
#include <msl_msgs/JoystickCommand.h>

namespace rqt_msl_joystick
{

	using namespace std;

	Joystick::Joystick() :
			rqt_gui_cpp::Plugin(), uiWidget(0)
	{
		setObjectName("Joystick");
		rosNode = new ros::NodeHandle();
		joyPub = rosNode->advertise<msl_msgs::JoystickCommand>("/Joystick", 1);
		spinner = new ros::AsyncSpinner(2);
		spinner->start();

		keyPressed = vector<bool>(6);
		for (int i = 0; i < keyPressed.size(); i++)
		{
			keyPressed[i] = false;
		}

		this->kick = false;
		this->robotId = 0;
		this->translation = 0;
		this->angle = 0;
		this->rotation = 0;
		this->kickPower = 0;
		this->shovelIdx = 0;
		this->ballHandleLeftMotor = 0;
		this->ballHandleRightMotor = 0;
		this->selectedActuator = 0;
	}

	void Joystick::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		//used to enable colored buttons
		QApplication::setStyle(new QPlastiqueStyle);
		uiWidget = new QWidget();
		setupUi(uiWidget);
		if (context.serialNumber() > 1)
		{
			uiWidget->setWindowTitle(uiWidget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
		}
		context.addWidget(uiWidget);

		// connect the edit fields
		connect(robotIdEdit, SIGNAL(returnPressed()), this, SLOT(onRobotIdEdited()));
		connect(kickPowerEdit, SIGNAL(returnPressed()), this, SLOT(onKickPowerEdited()));
		connect(translationEdit, SIGNAL(returnPressed()), this, SLOT(onTranslationEdited()));
		connect(rotationEdit, SIGNAL(returnPressed()), this, SLOT(onRotationEdited()));
		connect(ballHandleLeftEdit, SIGNAL(returnPressed()), this, SLOT(onBallHandleLeftEdited()));
		connect(ballHandleRightEdit, SIGNAL(returnPressed()), this, SLOT(onBallHandleRightEdited()));

		// connect the sliders
		connect(kickPowerSlider, SIGNAL(sliderReleased()), this, SLOT(onKickPowerSlided()));
		connect(translationSlider, SIGNAL(sliderReleased()), this, SLOT(onTranslationSlided()));
		connect(rotationSpeedSlider, SIGNAL(sliderReleased()), this, SLOT(onRotationSpeedSlided()));
		connect(ballHandleRightSlider, SIGNAL(sliderReleased()), this, SLOT(onBallHandleRightSlided()));
		connect(ballHandleLeftSlider, SIGNAL(sliderReleased()), this, SLOT(onBallHandleLeftSlided()));

		// connect the shovel radio button
		connect(lowShovelButton, SIGNAL(toggled(bool)), this, SLOT(onLowShovelSelected(bool)));
		connect(highShovelButton, SIGNAL(toggled(bool)), this, SLOT(onHighShovelSelected(bool)));
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

	void Joystick::sendJoystickMessage()
	{
		msl_msgs::JoystickCommand msg;
		msg.robotId = this->robotId;
		msg.ballHandleLeftMotor = this->ballHandleLeftMotor;
		msg.ballHandleRightMotor = this->ballHandleRightMotor;
		msg.kickPower = this->kickPower;
		msg.shovelIdx = this->shovelIdx;

		msg.selectedActuator = this->selectedActuator;

		msg.kick = this->kick;
		if (this->kick) // only shoot for one message
		{
			this->kick = false;
		}

		msg.motion.translation = this->translation;
		msg.motion.rotation = this->rotation;

		// driving direction
		// 0 == up
		// 1 == down
		// 2 == left
		// 3 == right

		if (keyPressed[0] == true && keyPressed[2] == true)
		{
			msg.motion.angle = (0.75 * M_PI);
		}
		else if (keyPressed[0] == true && keyPressed[3] == true)
		{
			msg.motion.angle = (1.25 * M_PI);
		}
		else if (keyPressed[1] == true && keyPressed[2] == true)
		{
			msg.motion.angle = (0.25 * M_PI);
		}
		else if (keyPressed[1] == true && keyPressed[3] == true)
		{
			msg.motion.angle = (1.75 * M_PI);
		}
		else if (keyPressed[0] == true)
		{
			msg.motion.angle = M_PI;
		}
		else if (keyPressed[1] == true)
		{
			msg.motion.angle = 0;
		}
		else if (keyPressed[2] == true)
		{
			msg.motion.angle = (1.5 * M_PI);
		}
		else if (keyPressed[3] == true)
		{
			msg.motion.angle = (0.5 * M_PI);
		}
		else
		{
			msg.motion.translation = 0;
			msg.motion.rotation = 0;
		}

		// rotation
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

		joyPub.publish(msg);
	}

	void Joystick::keyPressEvent(QKeyEvent* event)
	{
		if (!(event->isAutoRepeat()))
		{
			cout << "Joystick: Key pressed: " << event->key() << endl;

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
			cout << "Joystick: Key released: " << event->key() << endl;

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
		this->robotId = robotIdEdit->text().toInt();
		robotIdEdit->clearFocus();
		this->printControlValues();
	}

	void Joystick::onKickPowerEdited()
	{
		this->kickPower = kickPowerEdit->text().toShort();
		kickPowerEdit->clearFocus();
		this->printControlValues();
	}

	void Joystick::onKickPowerSlided()
	{
		this->kickPower = (short)kickPowerSlider->value();
		kickPowerSlider->clearFocus();
		this->printControlValues();
	}

	void Joystick::onRotationEdited()
	{
		this->rotation = rotationEdit->text().toDouble();
		rotationEdit->clearFocus();
		this->printControlValues();
	}

	void Joystick::onRotationSpeedSlided()
	{
		this->rotation = (double)rotationSpeedSlider->value();
		rotationSpeedSlider->clearFocus();
		this->printControlValues();
	}

	void Joystick::onTranslationEdited()
	{
		this->translation = translationEdit->text().toDouble();
		translationEdit->clearFocus();
		this->printControlValues();
	}

	void Joystick::onTranslationSlided()
	{
		this->translation = (double)translationSlider->value();
		translationSlider->clearFocus();
		this->printControlValues();
	}

	void Joystick::onLowShovelSelected(bool checked)
	{
		if (checked)
		{
			this->shovelIdx = 0;
			this->printControlValues();
		}
	}

	void Joystick::onHighShovelSelected(bool checked)
	{
		if (checked)
		{
			this->shovelIdx = 1;
			this->printControlValues();
		}
	}

	void Joystick::onBallHandleRightEdited()
	{
		this->ballHandleRightMotor = ballHandleRightEdit->text().toShort();
		ballHandleRightEdit->clearFocus();
		this->printControlValues();
	}

	void Joystick::onBallHandleRightSlided()
	{
		this->ballHandleRightMotor = (short) ballHandleRightSlider->value();
		ballHandleRightSlider->clearFocus();
		this->printControlValues();
	}

	void Joystick::onBallHandleLeftEdited()
	{
		this->ballHandleLeftMotor = ballHandleLeftEdit->text().toShort();
		ballHandleLeftEdit->clearFocus();
		this->printControlValues();
	}

	void Joystick::onBallHandleLeftSlided()
	{
		this->ballHandleLeftMotor = (short) ballHandleLeftSlider->value();
		ballHandleLeftSlider->clearFocus();
		this->printControlValues();
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

	/**
	 * Prints the current control values to the console.
	 */
	void Joystick::printControlValues()
	{
		cout << "--------- Control Values ----------" << endl;
		cout << "RobotId:\t" << this->robotId << endl;
		cout << "Translation:\t" << this->translation << endl;
		cout << "Angle:\t\t" << this->angle << endl;
		cout << "Rotation:\t" << this->rotation << endl;
		cout << "KickPower:\t" << this->kickPower << endl;
		cout << "Kick:\t\t" << (this->kick ? "True" : "False") << endl;
		cout << "Shovel:\t\t";
		if (this->shovelIdx > 0)
		{
			cout << "High" << endl;
		}
		else
		{
			cout << "Low" << endl;
		}
		cout << "BallHandle Left:" << this->ballHandleLeftMotor << "\t Right: " << this->ballHandleRightMotor << endl;
	}

} // namespace msl_keyboard_joystick

PLUGINLIB_EXPORT_CLASS(rqt_msl_joystick::Joystick, rqt_gui_cpp::Plugin)

