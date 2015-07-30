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

		connect(robotIdEdit, SIGNAL(returnPressed()), this, SLOT(onRobotIdEdited()));
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
		msg.ballHandleLeftMotor = this->ballHandleLeftMotor;
		msg.ballHandleRightMotor = this->ballHandleRightMotor;
		msg.kick = this->kick;
		msg.kickPower = this->kickPower;
		msg.robotId = this->robotId;
		msg.selectedActuator = this->selectedActuator;
		msg.shovelIdx = this->shovelIdx;
		msg.motion.translation = this->translation;
		msg.motion.rotation = this->rotation;

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
		joyPub.publish(msg);
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
		this->robotId = robotIdEdit->text().toInt();
		robotIdEdit->clearFocus();
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

} // namespace msl_keyboard_joystick

PLUGINLIB_EXPORT_CLASS(rqt_msl_joystick::Joystick, rqt_gui_cpp::Plugin)

