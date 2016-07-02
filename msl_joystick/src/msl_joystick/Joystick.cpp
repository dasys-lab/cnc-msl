#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <msl_joystick/Joystick.h>
#include <msl_msgs/JoystickCommand.h>
#include <SystemConfig.h>
#include <limits>

namespace msl_joystick
{

	using namespace std;

	Joystick::Joystick() :
			rqt_gui_cpp::Plugin(), uiWidget(0), sendMsgTimer(0)
	{
		setObjectName("Joystick");

		auto sc = supplementary::SystemConfig::getInstance();
		ballHandleMin = (*sc)["Joystick"]->get<short>("Joystick.ballHandleMin", NULL);
		ballHandleMax = (*sc)["Joystick"]->get<short>("Joystick.ballHandleMax", NULL);
		kickPowerMin = (*sc)["Joystick"]->get<short>("Joystick.kickPowerMin", NULL);
		kickPowerMax = (*sc)["Joystick"]->get<short>("Joystick.kickPowerMax", NULL);
		translationMin = (*sc)["Joystick"]->get<double>("Joystick.translationMin", NULL);
		translationMax = (*sc)["Joystick"]->get<double>("Joystick.translationMax", NULL);
		rotationMin = (*sc)["Joystick"]->get<double>("Joystick.rotationMin", NULL);
		rotationMax = (*sc)["Joystick"]->get<double>("Joystick.rotationMax", NULL);
		sendInterval = (*sc)["Joystick"]->get<int>("Joystick.sendInterval", NULL);
		string joystickCmdTopic = (*sc)["Joystick"]->get<string>("Joystick.Topics.joystickCmdTopic", NULL);

		rosNode = new ros::NodeHandle();
		joyPub = rosNode->advertise<msl_msgs::JoystickCommand>(joystickCmdTopic, 1);
		spinner = new ros::AsyncSpinner(2);
		spinner->start();

		keyPressed = vector<bool>(8);
		for (int i = 0; i < keyPressed.size(); i++)
		{
			keyPressed[i] = false;
		}

		this->robotId = -1;
		this->translation = translationMin;
		this->rotation = rotationMin;
		this->kickPower = kickPowerMin;
		this->shovelIdx = 0;
		this->ballHandleLeftMotor = 0;
		this->ballHandleRightMotor = 0;
		this->useBallHandle = msl_msgs::JoystickCommand::BALL_HANDLE_OFF;
	}

	void Joystick::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		uiWidget = new QWidget();
		uiWidget->installEventFilter(this);
		uiWidget->setFocusPolicy(Qt::FocusPolicy::ClickFocus);
		uiWidget->setAttribute(Qt::WA_AlwaysShowToolTips, true);
		setupUi(uiWidget);
		if (context.serialNumber() > 1)
		{
			uiWidget->setWindowTitle(uiWidget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
		}
		context.addWidget(uiWidget);

		// set min and max values of slider according to conf file
		kickPowerSlider->setMinimum(this->kickPowerMin);
		kickPowerSlider->setMaximum(this->kickPowerMax);
		ballHandleLeftSlider->setMinimum(this->ballHandleMin);
		ballHandleLeftSlider->setMaximum(this->ballHandleMax);
		ballHandleRightSlider->setMinimum(this->ballHandleMin);
		ballHandleRightSlider->setMaximum(this->ballHandleMax);

		// connect the edit fields
		connect(robotIdEdit, SIGNAL(editingFinished()), this, SLOT(onRobotIdEdited()));
		connect(kickPowerEdit, SIGNAL(editingFinished()), this, SLOT(onKickPowerEdited()));
		connect(translationEdit, SIGNAL(editingFinished()), this, SLOT(onTranslationEdited()));
		connect(rotationEdit, SIGNAL(editingFinished()), this, SLOT(onRotationEdited()));
		connect(ballHandleLeftEdit, SIGNAL(editingFinished()), this, SLOT(onBallHandleLeftEdited()));
		connect(ballHandleRightEdit, SIGNAL(editingFinished()), this, SLOT(onBallHandleRightEdited()));

		// connect the sliders
		connect(kickPowerSlider, SIGNAL(valueChanged(int)), this, SLOT(onKickPowerSlided(int)));
		connect(ballHandleRightSlider, SIGNAL(valueChanged(int)), this, SLOT(onBallHandleRightSlided(int)));
		connect(ballHandleLeftSlider, SIGNAL(valueChanged(int)), this, SLOT(onBallHandleLeftSlided(int)));

		// connect the shovel radio button and ballHandle state
		connect(lowShovelButton, SIGNAL(toggled(bool)), this, SLOT(onLowShovelSelected(bool)));
		connect(highShovelButton, SIGNAL(toggled(bool)), this, SLOT(onHighShovelSelected(bool)));
		connect(ballHandleStateCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onBallHandleCheckBoxToggled(int)));

		sendMsgTimer = new QTimer(this);
		connect(sendMsgTimer, SIGNAL(timeout()), this, SLOT(sendJoystickMessage()));
		sendMsgTimer->start(this->sendInterval);
	}

	void Joystick::shutdownPlugin()
	{
		delete spinner;
		delete rosNode;
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
		if (!(keyPressed[0] || keyPressed[1] || keyPressed[2] || keyPressed[3] || keyPressed[4] || keyPressed[5]
				|| keyPressed[6] || keyPressed[7]))
		{
			return; // dont send joystick message if no key is pressed
		}

		if (!this->uiWidget->hasFocus())
		{
			for (int i = 0; i < keyPressed.size(); i++)
			{
				keyPressed[i] = false; // just to be sure, in case of missed key released events
			}
			return; // dont send joystick message if joystick window does not have the focus
		}

		msl_msgs::JoystickCommand msg;

		// robotid
		msg.robotId = this->robotId;

		// ballHandle stuff
		msg.ballHandleLeftMotor = this->ballHandleLeftMotor;
		msg.ballHandleRightMotor = this->ballHandleRightMotor;
		if (this->useBallHandle)
		{
			msg.ballHandleState = msl_msgs::JoystickCommand::BALL_HANDLE_ON;
		}
		else
		{
			msg.ballHandleState = msl_msgs::JoystickCommand::BALL_HANDLE_OFF;
		}

		// kicker stuff
		msg.kickPower = this->kickPower;
		if (this->keyPressed[6] == true)
		{
			msg.kick = true;
			this->keyPressed[6] = false; // only shoot for one message
		}
		else
		{
			msg.kick = false;
		}

		msg.shovelIdx = this->shovelIdx;
		if (this->keyPressed[7] == true)
		{
			this->keyPressed[7] = false;
		}

		if (!(keyPressed[0] || keyPressed[1] || keyPressed[2] || keyPressed[3] || keyPressed[4] || keyPressed[5]))
		{
			// Send NaN to signal Joystick behaviour NOT to send MotionControl commands.
			msg.motion.translation = std::numeric_limits<double>::quiet_NaN();
			msg.motion.rotation = std::numeric_limits<double>::quiet_NaN();
			msg.motion.angle = std::numeric_limits<double>::quiet_NaN();
		}
		else
		{
			// translation
			msg.motion.translation = this->translation;

			// driving direction
			// 0 == up
			// 1 == down
			// 2 == left
			// 3 == right

			// angle
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
		}

#ifdef RQT_MSL_JOYSTICK_DEBUG
		this->printControlValues();
		//this->printJoystickMessage(msg);
#endif
		joyPub.publish(msg);
	}

	/**
	 * Handles KeyPress and KeyRelease Events by forwarding them to the corresponding method.
	 * Other Events are ignored by calling QObject:eventFilter()
	 */
	bool Joystick::eventFilter(QObject* watched, QEvent* event)
	{
		if (watched == uiWidget)
		{
			if (event->type() == QEvent::KeyPress)
			{
				this->keyPressEvent(static_cast<QKeyEvent *>(event));
				return true;
			}
			if (event->type() == QEvent::KeyRelease)
			{
				this->keyReleaseEvent(static_cast<QKeyEvent *>(event));
				return true;
			}
		}

		return QObject::eventFilter(watched, event);
	}

	void Joystick::keyPressEvent(QKeyEvent* event)
	{
		if (!(event->isAutoRepeat()))
		{
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
					keyPressed[6] = true;
					break;
				default:
					break;
			}
		}
	}

	void Joystick::keyReleaseEvent(QKeyEvent* event)
	{
		if (!(event->isAutoRepeat()))
		{
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
					keyPressed[6] = false;
					break;
				default:
					break;
			}
		}
	}

	// QT GUI Signal and Slots Methods

	void Joystick::onRobotIdEdited()
	{
		this->robotId = robotIdEdit->text().toInt();
		this->robotIdEdit->setText(QString::number(this->robotId));
		this->uiWidget->setFocus();
	}

	void Joystick::onKickPowerEdited()
	{
		this->kickPower = kickPowerEdit->text().toShort();
		if (this->kickPower > this->kickPowerMax)
			this->kickPower = this->kickPowerMax;
		if (this->kickPower < this->kickPowerMin)
			this->kickPower = this->kickPowerMin;
		this->kickPowerEdit->setText(QString::number(this->kickPower));
		bool result = this->kickPowerSlider->blockSignals(true);
		this->kickPowerSlider->setValue(this->kickPower);
		this->kickPowerSlider->blockSignals(result);
		this->uiWidget->setFocus();
	}

	void Joystick::onKickPowerSlided(int value)
	{
		this->kickPower = (short)value;
		this->kickPowerEdit->setText(QString::number(value));
		this->uiWidget->setFocus();
	}

	void Joystick::onRotationEdited()
	{
		this->rotation = rotationEdit->text().toDouble();
		if (this->rotation > this->rotationMax)
			this->rotation = this->rotationMax;
		if (this->rotation < this->rotationMin)
			this->rotation = this->rotationMin;
		this->rotationEdit->setText(QString::number(this->rotation));
		this->uiWidget->setFocus();
	}

	void Joystick::onTranslationEdited()
	{
		this->translation = translationEdit->text().toDouble();
		if (this->translation > this->translationMax)
			this->translation = this->translationMax;
		if (this->translation < this->translationMin)
			this->translation = this->translationMin;
		this->translationEdit->setText(QString::number(this->translation));
		this->uiWidget->setFocus();
	}

	void Joystick::onLowShovelSelected(bool checked)
	{
		if (checked)
		{
			this->shovelIdx = 0;
			this->keyPressed[7] = true; // signal to send joystick message for changing shovel
			this->uiWidget->setFocus();
		}
	}

	void Joystick::onHighShovelSelected(bool checked)
	{
		if (checked)
		{
			this->shovelIdx = 1;
			this->keyPressed[7] = true; // signal to send joystick message for changing shovel
			this->uiWidget->setFocus();
		}
	}

	void Joystick::onBallHandleRightEdited()
	{
		this->ballHandleRightMotor = ballHandleRightEdit->text().toInt();
		if (this->ballHandleRightMotor > this->ballHandleMax)
			this->ballHandleRightMotor = this->ballHandleMax;
		if (this->ballHandleRightMotor < this->ballHandleMin)
			this->ballHandleRightMotor = this->ballHandleMin;
		this->ballHandleRightEdit->setText(QString::number(this->ballHandleRightMotor));
		bool result = this->ballHandleRightSlider->blockSignals(true);
		this->ballHandleRightSlider->setValue(this->ballHandleRightMotor);
		this->ballHandleRightSlider->blockSignals(result);
		this->uiWidget->setFocus();
	}

	void Joystick::onBallHandleRightSlided(int value)
	{
		this->ballHandleRightMotor = value;
		ballHandleRightEdit->setText(QString::number(value));
		this->uiWidget->setFocus();
	}

	void Joystick::onBallHandleLeftEdited()
	{
		this->ballHandleLeftMotor = ballHandleLeftEdit->text().toInt();
		if (this->ballHandleLeftMotor > this->ballHandleMax)
			this->ballHandleLeftMotor = this->ballHandleMax;
		if (this->ballHandleLeftMotor < this->ballHandleMin)
			this->ballHandleLeftMotor = this->ballHandleMin;
		this->ballHandleLeftEdit->setText(QString::number(this->ballHandleLeftMotor));
		bool result = this->ballHandleLeftSlider->blockSignals(true);
		this->ballHandleLeftSlider->setValue(this->ballHandleLeftMotor);
		this->ballHandleLeftSlider->blockSignals(result);
		this->uiWidget->setFocus();
	}

	void Joystick::onBallHandleLeftSlided(int value)
	{
		this->ballHandleLeftMotor = value;
		this->ballHandleLeftEdit->setText(QString::number(value));
		this->uiWidget->setFocus();
	}

	void Joystick::onBallHandleCheckBoxToggled(int checkState)
	{
		switch (checkState)
		{
			case Qt::CheckState::Checked:
				std::cout << "BHC: set to true" << endl;
				this->useBallHandle = true;
				break;
			case Qt::CheckState::Unchecked:
			case Qt::CheckState::PartiallyChecked:
			default:
				std::cout << "BHC: set to false" << endl;
				this->useBallHandle = false;
				break;
		}
		this->uiWidget->setFocus();
	}

	/**
	 * Prints the current control values to the console.
	 */
	void Joystick::printControlValues()
	{
		cout << "------------ Control Values -------------" << endl;
		cout << "RobotId:\t\t" << this->robotId << endl;
		cout << "Translation:\t\t" << this->translation << endl;
		cout << "Rotation:\t\t" << this->rotation << endl;
		cout << "KickPower:\t\t" << this->kickPower << endl;
		cout << "Shovel:\t\t\t";
		if (this->shovelIdx > 0)
		{
			cout << "High" << endl;
		}
		else
		{
			cout << "Low" << endl;
		}
		cout << "BallHandle State:\t" << (this->useBallHandle ? "On" : "Off") << endl;
		cout << "BallHandle Left:\t" << this->ballHandleLeftMotor << "\t Right: " << this->ballHandleRightMotor << endl;
	}

	/**
	 * Prints the current joystick command values to the console.
	 */
	void Joystick::printJoystickMessage(msl_msgs::JoystickCommand msg)
	{
		cout << "------------ Command Values -------------" << endl;
		cout << "RobotId:\t\t" << msg.robotId << endl;
		cout << "Translation:\t\t" << msg.motion.translation << endl;
		cout << "Angle:\t\t\t" << msg.motion.angle << endl;
		cout << "Rotation:\t\t" << msg.motion.rotation << endl;
		cout << "KickPower:\t\t" << msg.kickPower << endl;
		cout << "Kick:\t\t\t" << (msg.kick ? "True" : "False") << endl;
		cout << "Shovel:\t\t\t";
		if (msg.shovelIdx > 0)
		{
			cout << "High" << endl;
		}
		else
		{
			cout << "Low" << endl;
		}
		cout << "BallHandle State:\t" << (msg.ballHandleState ? "On" : "Off") << endl;
		cout << "BallHandle Left:\t" << (int)msg.ballHandleLeftMotor << "\t Right: "
				<< (int)msg.ballHandleRightMotor << endl;
	}

} // namespace msl_keyboard_joystick

PLUGINLIB_EXPORT_CLASS(msl_joystick::Joystick, rqt_gui_cpp::Plugin)

