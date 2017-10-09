#include <SystemConfig.h>
#include <limits>
#include <msl_joystick/Joystick.h>
#include <msl_msgs/JoystickCommand.h>
#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <math.h>

namespace msl_joystick
{

    using namespace std;

    Joystick::Joystick() :
            rqt_gui_cpp::Plugin(), uiWidget(0), sendMsgTimer(0), gamePadTimer(0)
    {
        setObjectName("Joystick");

        auto sc = supplementary::SystemConfig::getInstance();
        this->ballHandleMin = (*sc)["Joystick"]->get<short>("Joystick.ballHandleMin", NULL);
        this->ballHandleMax = (*sc)["Joystick"]->get<short>("Joystick.ballHandleMax", NULL);
        this->kickPowerMin = (*sc)["Joystick"]->get<short>("Joystick.kickPowerMin", NULL);
        this->kickPowerMax = (*sc)["Joystick"]->get<short>("Joystick.kickPowerMax", NULL);
        this->translationMin = (*sc)["Joystick"]->get<double>("Joystick.translationMin", NULL);
        this->translationMax = (*sc)["Joystick"]->get<double>("Joystick.translationMax", NULL);
        this->rotationMin = (*sc)["Joystick"]->get<double>("Joystick.rotationMin", NULL);
        this->rotationMax = (*sc)["Joystick"]->get<double>("Joystick.rotationMax", NULL);
        this->sendInterval = (*sc)["Joystick"]->get<int>("Joystick.sendInterval", NULL);
        string joystickCmdTopic = (*sc)["Joystick"]->get<string>("Joystick.Topics.joystickCmdTopic", NULL);

        this->rosNode = new ros::NodeHandle();
        this->joyPub = rosNode->advertise<msl_msgs::JoystickCommand>(joystickCmdTopic, 1);
        this->joySub = rosNode->subscribe("/joy", 10, &Joystick::onJoyMsg, (Joystick *)this);
        this->spinner = new ros::AsyncSpinner(2);
        this->spinner->start();

        this->keyPressed = vector<bool>(8);
        for (int i = 0; i < this->keyPressed.size(); i++)
        {
            this->keyPressed[i] = false;
        }

        this->robotId = -1;
        this->translation = this->translationMin;
        this->rotation = this->rotationMin;
        this->kickPower = this->kickPowerMin;
        this->shovelIdx = 0;
        this->ballHandleLeftMotor = 0;
        this->ballHandleRightMotor = 0;
        this->useBallHandle = msl_msgs::JoystickCommand::BALL_HANDLE_OFF;
        this->usePTController = msl_msgs::JoystickCommand::PT_CONTROLLER_OFF;

        this->useGamePad = false;
        this->joyNodePID = -1;
        this->joyExec = "/opt/ros/kinetic/lib/joy/joy_node";
        this->kickIncrease = 100;
        this->translationIncrease = 100;
        this->rotationIncrease = 1;
        this->dribbleManually = false;
        this->ballHandleSign = 1;
        this->ltPressedOnce = false;
        this->rtPressedOnce = false;
    }

    void Joystick::initPlugin(qt_gui_cpp::PluginContext &context)
    {
        this->uiWidget = new QWidget();
        this->uiWidget->installEventFilter(this);
        this->uiWidget->setFocusPolicy(Qt::FocusPolicy::ClickFocus);
        this->uiWidget->setAttribute(Qt::WA_AlwaysShowToolTips, true);
        setupUi(uiWidget);
        if (context.serialNumber() > 1)
        {
            this->uiWidget->setWindowTitle(
                    this->uiWidget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }
        context.addWidget(this->uiWidget);

        // set min and max values of slider according to conf file
        this->kickPowerSlider->setMinimum(this->kickPowerMin);
        this->kickPowerSlider->setMaximum(this->kickPowerMax);
        this->ballHandleLeftSlider->setMinimum(this->ballHandleMin);
        this->ballHandleLeftSlider->setMaximum(this->ballHandleMax);
        this->ballHandleRightSlider->setMinimum(this->ballHandleMin);
        this->ballHandleRightSlider->setMaximum(this->ballHandleMax);

        // connect the edit fields
        connect(this->robotIdEdit, SIGNAL(editingFinished()), this, SLOT(onRobotIdEdited()));
        connect(this->kickPowerEdit, SIGNAL(editingFinished()), this, SLOT(onKickPowerEdited()));
        connect(this->translationEdit, SIGNAL(editingFinished()), this, SLOT(onTranslationEdited()));
        connect(this->rotationEdit, SIGNAL(editingFinished()), this, SLOT(onRotationEdited()));
        connect(this->ballHandleLeftEdit, SIGNAL(editingFinished()), this, SLOT(onBallHandleLeftEdited()));
        connect(this->ballHandleRightEdit, SIGNAL(editingFinished()), this, SLOT(onBallHandleRightEdited()));

        // connect the sliders
        connect(this->kickPowerSlider, SIGNAL(valueChanged(int)), this, SLOT(onKickPowerSlided(int)));
        connect(this->ballHandleRightSlider, SIGNAL(valueChanged(int)), this, SLOT(onBallHandleRightSlided(int)));
        connect(this->ballHandleLeftSlider, SIGNAL(valueChanged(int)), this, SLOT(onBallHandleLeftSlided(int)));

        // connect the shovel radio button and ballHandle state
        connect(this->lowShovelButton, SIGNAL(toggled(bool)), this, SLOT(onLowShovelSelected(bool)));
        connect(this->highShovelButton, SIGNAL(toggled(bool)), this, SLOT(onHighShovelSelected(bool)));
        connect(this->ballHandleStateCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onBallHandleCheckBoxToggled(int)));

        this->sendMsgTimer = new QTimer(this);
        connect(this->sendMsgTimer, SIGNAL(timeout()), this, SLOT(sendJoystickMessage()));
        this->sendMsgTimer->start(this->sendInterval);

        // connect gamepad
        connect(useGamePadCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onUseGamePadCheckBoxToggled(int)));
        connect(this, SIGNAL(toggleShovelSelect(bool)), this, SLOT(onToggleShovel(bool)));

        this->gamePadTimer = new QTimer(this);
        connect(this->gamePadTimer, SIGNAL(timeout()), this, SLOT(resendJoyCmd()));
        connect(this, SIGNAL(stopJoyTimer()), this->gamePadTimer, SLOT(stop()));
        connect(this, SIGNAL(startJoyTimer()), this->gamePadTimer, SLOT(start()));
        connect(this, SIGNAL(toggleBallHandle()), this, SLOT(onToggleBallHandle()));
        connect(this, SIGNAL(toggleUsePt()), this, SLOT(onToggleUsePt()));
        connect(this, SIGNAL(changeKickPower(int)), this, SLOT(onKickPowerChanged(int)));
        connect(this, SIGNAL(changeRotation(int)), this, SLOT(onRotationChanged(int)));
        connect(this, SIGNAL(changeTranslation(int)), this, SLOT(onTranslationChanged(int)));
    }

    void Joystick::shutdownPlugin()
    {
        if (this->joyNodePID != -1)
        {
            cout << "Joystick: Try to kill " << this->joyNodePID << endl;
            kill(this->joyNodePID, SIGTERM);
            this->joyNodePID = -1;
        }
        delete this->sendMsgTimer;
        delete this->gamePadTimer;
        delete this->spinner;
        delete this->rosNode;
    }

    void Joystick::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
    {
    }

    void Joystick::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                   const qt_gui_cpp::Settings &instance_settings)
    {
    }

    void Joystick::sendJoystickMessage()
    {
        if (!(this->keyPressed[0] || this->keyPressed[1] || this->keyPressed[2] || this->keyPressed[3]
                || this->keyPressed[4] || this->keyPressed[5] || this->keyPressed[6] || this->keyPressed[7]))
        {
            return; // dont send joystick message if no key is pressed
        }

        if (!this->uiWidget->hasFocus())
        {
            for (int i = 0; i < this->keyPressed.size(); i++)
            {
                this->keyPressed[i] = false; // just to be sure, in case of missed key released events
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
            if (this->keyPressed[0] == true && this->keyPressed[2] == true)
            {
                msg.motion.angle = (1.25 * M_PI);
            }
            else if (this->keyPressed[0] == true && this->keyPressed[3] == true)
            {
                msg.motion.angle = (0.75 * M_PI);
            }
            else if (this->keyPressed[1] == true && this->keyPressed[2] == true)
            {
                msg.motion.angle = (1.75 * M_PI);
            }
            else if (this->keyPressed[1] == true && this->keyPressed[3] == true)
            {
                msg.motion.angle = (0.25 * M_PI);
            }
            else if (this->keyPressed[0] == true)
            {
                msg.motion.angle = M_PI;
            }
            else if (this->keyPressed[1] == true)
            {
                msg.motion.angle = 0;
            }
            else if (this->keyPressed[2] == true)
            {
                msg.motion.angle = (1.5 * M_PI);
            }
            else if (this->keyPressed[3] == true)
            {
                msg.motion.angle = (0.5 * M_PI);
            }
            else
            {
                msg.motion.translation = 0;
                msg.motion.rotation = 0;
            }

            // rotation
            if (this->keyPressed[4] == true)
            {
                msg.motion.rotation = this->rotation;
            }
            else if (this->keyPressed[5] == true)
            {
                msg.motion.rotation = -this->rotation;
            }
            else
            {
                msg.motion.rotation = 0;
            }
        }
#ifdef RQT_MSL_JOYSTICK_DEBUG
        this->printControlValues();
// this->printJoystickMessage(msg);
#endif
        this->joyPub.publish(msg);
    }

    /**
     * Handles KeyPress and KeyRelease Events by forwarding them to the corresponding method.
     * Other Events are ignored by calling QObject:eventFilter()
     */
    bool Joystick::eventFilter(QObject *watched, QEvent *event)
    {
        if (watched == this->uiWidget)
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

    void Joystick::keyPressEvent(QKeyEvent *event)
    {
        if (!(event->isAutoRepeat()))
        {
            switch (event->key())
            {
                case Qt::Key_Up:
                    this->keyPressed[0] = true;
                    break;
                case Qt::Key_Down:
                    this->keyPressed[1] = true;
                    break;
                case Qt::Key_Left:
                    this->keyPressed[2] = true;
                    break;
                case Qt::Key_Right:
                    this->keyPressed[3] = true;
                    break;
                case Qt::Key_Less:
                    this->keyPressed[4] = true;
                    break;
                case Qt::Key_Y:
                    this->keyPressed[5] = true;
                    break;
                case Qt::Key_Space:
                    this->keyPressed[6] = true;
                    break;
                default:
                    break;
            }
        }
    }

    void Joystick::keyReleaseEvent(QKeyEvent *event)
    {
        if (!(event->isAutoRepeat()))
        {
            switch (event->key())
            {
                case Qt::Key_Up:
                    this->keyPressed[0] = false;
                    break;
                case Qt::Key_Down:
                    this->keyPressed[1] = false;
                    break;
                case Qt::Key_Left:
                    this->keyPressed[2] = false;
                    break;
                case Qt::Key_Right:
                    this->keyPressed[3] = false;
                    break;
                case Qt::Key_Less:
                    this->keyPressed[4] = false;
                    break;
                case Qt::Key_Y:
                    this->keyPressed[5] = false;
                    break;
                case Qt::Key_Space:
                    this->keyPressed[6] = false;
                    break;
                default:
                    break;
            }
        }
    }

// QT GUI Signal and Slots Methods

    void Joystick::onRobotIdEdited()
    {
        this->robotId = this->robotIdEdit->text().toInt();
        this->robotIdEdit->setText(QString::number(this->robotId));
        this->uiWidget->setFocus();
    }

    void Joystick::onKickPowerEdited()
    {
        this->kickPower = this->kickPowerEdit->text().toShort();
        if (this->kickPower > this->kickPowerMax)
        {
            this->kickPower = this->kickPowerMax;
        }
        if (this->kickPower < this->kickPowerMin)
        {
            this->kickPower = this->kickPowerMin;
        }
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
        this->rotation = this->rotationEdit->text().toDouble();
        if (this->rotation > this->rotationMax)
        {
            this->rotation = this->rotationMax;
        }
        if (this->rotation < this->rotationMin)
        {
            this->rotation = this->rotationMin;
        }
        this->rotationEdit->setText(QString::number(this->rotation));
        this->uiWidget->setFocus();
    }

    void Joystick::onTranslationEdited()
    {
        this->translation = this->translationEdit->text().toDouble();
        if (this->translation > this->translationMax)
        {
            this->translation = this->translationMax;
        }
        if (this->translation < this->translationMin)
        {
            this->translation = this->translationMin;
        }
        this->translationEdit->setText(QString::number(this->translation));
        this->uiWidget->setFocus();
    }

    void Joystick::onLowShovelSelected(bool checked)
    {
        if (checked)
        {
            this->shovelIdx = 0;
            if (!this->useGamePad)
            {
                this->keyPressed[7] = true; // signal to send joystick message for changing shovel
            }
            this->uiWidget->setFocus();
        }
    }

    void Joystick::onHighShovelSelected(bool checked)
    {
        if (checked)
        {
            this->shovelIdx = 1;
            if (!this->useGamePad)
            {
                this->keyPressed[7] = true; // signal to send joystick message for changing shovel
            }
            this->uiWidget->setFocus();
        }
    }

    void Joystick::onBallHandleRightEdited()
    {
        this->ballHandleRightMotor = this->ballHandleRightEdit->text().toInt();
        if (this->ballHandleRightMotor > this->ballHandleMax)
        {
            this->ballHandleRightMotor = this->ballHandleMax;
        }
        if (this->ballHandleRightMotor < this->ballHandleMin)
        {
            this->ballHandleRightMotor = this->ballHandleMin;
        }
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
        this->ballHandleLeftMotor = this->ballHandleLeftEdit->text().toInt();
        if (this->ballHandleLeftMotor > this->ballHandleMax)
        {
            this->ballHandleLeftMotor = this->ballHandleMax;
        }
        if (this->ballHandleLeftMotor < this->ballHandleMin)
        {
            this->ballHandleLeftMotor = this->ballHandleMin;
        }
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
                this->useBallHandle = msl_msgs::JoystickCommand::BALL_HANDLE_ON;
                if (!this->dribbleManually)
                {
                    this->manual_label->setText("automatic");
                }
                break;
            case Qt::CheckState::Unchecked:
            case Qt::CheckState::PartiallyChecked:
            default:
                std::cout << "BHC: set to false" << endl;
                this->useBallHandle = msl_msgs::JoystickCommand::BALL_HANDLE_OFF;
                this->manual_label->setText("");
                break;
        }
        this->uiWidget->setFocus();
    }

    void Joystick::onPTControllerCheckBoxToggled(int checkState)
    {
        switch (checkState)
        {
            case Qt::CheckState::Checked:
                std::cout << "PTC: set to true" << endl;
                this->usePTController = msl_msgs::JoystickCommand::PT_CONTROLLER_ON;
                break;
            case Qt::CheckState::Unchecked:
            case Qt::CheckState::PartiallyChecked:
            default:
                std::cout << "PTC: set to false" << endl;
                this->usePTController = msl_msgs::JoystickCommand::PT_CONTROLLER_OFF;
                break;
        }
        this->uiWidget->setFocus();
    }

    void Joystick::onUseGamePadCheckBoxToggled(int checkState)
    {
        vector<char*> startParams;
        char* tmp = new char[this->joyExec.size() + 1];
        strcpy(tmp, this->joyExec.c_str());
        tmp[this->joyExec.size() + 1] = '\0';
        startParams.push_back(tmp);
        startParams.push_back(nullptr);
        switch (checkState)
        {
            case Qt::CheckState::Checked:
                std::cout << "Use GamePad: set to true" << endl;
                this->useGamePad = true;
                int execReturn;
                this->joyNodePID = fork();
                if (this->joyNodePID == 0)
                {
                    execReturn = execvp("/opt/ros/kinetic/lib/joy/joy_node", startParams.data());
                    if (execReturn == -1)
                    {
                        cout << "Joystick: Failure! execve error code = " << errno << " - " << strerror(errno) << endl;
                        abort();
                    }
                }
                break;
            case Qt::CheckState::Unchecked:
            case Qt::CheckState::PartiallyChecked:
            default:
                std::cout << "Use GamePad: set to false" << endl;
                this->useGamePad = false;
                if (this->joyNodePID != -1)
                {
                    cout << "Joystick: Try to kill " << this->joyNodePID << endl;
                    kill(this->joyNodePID, SIGTERM);
                    this->joyNodePID = -1;
                    this->direction_label->setText("");
                    this->manual_label->setText("");
                }
                break;
        }
        this->uiWidget->setFocus();
    }

    void Joystick::onToggleShovel(bool shovel)
    {
        if (shovel)
        {
            this->lowShovelButton->setChecked(true);
        }
        else
        {
            this->highShovelButton->setChecked(true);
        }
    }

    void Joystick::onToggleBallHandle()
    {
        if (this->ballHandleStateCheckBox->isChecked())
        {
            this->ballHandleStateCheckBox->setChecked(false);
            this->manual_label->setText("");
        }
        else
        {
            this->ballHandleStateCheckBox->setChecked(true);
            this->manual_label->setText("automatic");
        }
    }

    void Joystick::onToggleUsePt()
    {
        if (this->ptControllerStateCheckBox->isChecked())
        {
            this->ptControllerStateCheckBox->setChecked(false);
        }
        else
        {
            this->ptControllerStateCheckBox->setChecked(true);
        }
    }

    void Joystick::onKickPowerChanged(int value)
    {
        if (this->kickPower + value > this->kickPowerMax)
        {
            this->kickPower = this->kickPowerMax;
        }
        else if (this->kickPower + value < this->kickPowerMin)
        {
            this->kickPower = this->kickPowerMin;
        }
        else
        {
            this->kickPower += value;
        }
        this->kickPowerEdit->setText(QString(to_string((int)this->kickPower).c_str()));
        this->kickPowerSlider->setValue((int)this->kickPower);
    }

    void Joystick::onTranslationChanged(int value)
    {
        if (this->translation + value > this->translationMax)
        {
            this->translation = this->translationMax;
        }
        else if (this->translation + value < this->translationMin)
        {
            this->translation = this->translationMin;
        }
        else
        {
            this->translation += value;
        }
        this->translationEdit->setText(QString(to_string((int)this->translation).c_str()));
    }

    void Joystick::onRotationChanged(int value)
    {
        if (this->rotation + value > this->rotationMax)
        {
            this->rotation = this->rotationMax;
        }
        else if (this->rotation + value < this->rotationMin)
        {
            this->rotation = this->rotationMin;
        }
        else
        {
            this->rotation += value;
        }
        this->rotationEdit->setText(QString(to_string(this->rotation).c_str()));
    }

    void Joystick::resendJoyCmd()
    {
        this->joyPub.publish(this->joycmd);
    }

    void Joystick::onJoyMsg(sensor_msgs::JoyPtr msg)
    {
        emit stopJoyTimer();

        // lb button => dead-man
        if (msg->buttons.at(4) == 0)
        {
            return; // dont send joystick message if dead-man is not pressed
        }

        // both axes are set to 0.0 if they have not been triggered
        // after first trigger they are set to 1.0 ...
        if (msg->axes.at(2) != 0)
        {
            this->ltPressedOnce = true;
        }

        if (msg->axes.at(5) != 0)
        {
            this->rtPressedOnce = true;
        }

        msl_msgs::JoystickCommand cmd = msl_msgs::JoystickCommand();

        cmd.robotId = this->robotId;

        // a button => decrease kick power
        if (msg->buttons.at(0) == 1)
        {
            emit changeKickPower(-this->kickIncrease);
        }

        // b button => select low shovel
        if (msg->buttons.at(1) == 1)
        {
            emit toggleShovelSelect(true);
        }

        // x button => select high shovel
        if (msg->buttons.at(2) == 1)
        {
            emit toggleShovelSelect(false);
        }

        //y button => increase kick power
        if (msg->buttons.at(3) == 1)
        {
            emit changeKickPower(this->kickIncrease);
        }

        //Button 4 is used as dead-man therefore used before

        // rb button => kick
        if (msg->buttons.at(5) == 1)
        {
            cmd.kick = true;
        }
        else
        {
            cmd.kick = false;
        }

        // back button => use pt controller on/off
        if (msg->buttons.at(6) == 1)
        {
            emit toggleUsePt();
        }

        // start button => use ballhandle on/off
        if (msg->buttons.at(7) == 1)
        {
            emit toggleBallHandle();
        }

        // logitech button => use ballhandle manually on/off
        if (msg->buttons.at(8) == 1)
        {
            if (!this->ballHandleStateCheckBox->isChecked())
            {
                this->ballHandleStateCheckBox->setChecked(true);
            }
            this->dribbleManually = !this->dribbleManually;
            if (this->dribbleManually)
            {
                this->manual_label->setText("manual:  ");
                this->direction_label->setText((this->ballHandleSign == 1) ? "backward" : "forward");
            }
            else
            {
                this->direction_label->setText("");
                if(this->ballHandleStateCheckBox->isChecked())
                {
                    this->manual_label->setText("automatic");
                }
                else
                {
                    this->manual_label->setText("");
                }
            }
        }

        // logitech button => use ballhandle manually on/off
        if (msg->buttons.at(9) == 1)
        {
            this->ballHandleSign = this->ballHandleSign * -1;
            this->direction_label->setText((this->ballHandleSign == 1) ? "backward" : "forward");
        }

        // button 10 (right stick) is not used

        // left stick (axes 0/1) => drive &&  right stick (axis 3) => rotation && axis 4 is not used
        if (abs(msg->axes.at(0)) == 0.0 && abs(msg->axes.at(1)) == 0.0 && abs(msg->axes.at(3)) == 0.0)
        {
            // Send NaN to signal Joystick behaviour NOT to send MotionControl commands.
            cmd.motion.translation = std::numeric_limits<double>::quiet_NaN();
            cmd.motion.rotation = std::numeric_limits<double>::quiet_NaN();
            cmd.motion.angle = std::numeric_limits<double>::quiet_NaN();
        }
        else
        {

            auto trans = sqrt(pow(msg->axes.at(0), 2) + pow(msg->axes.at(1), 2)) * this->translation;
            if (trans < this->translation)
            {
                cmd.motion.translation = trans;
            }
            else
            {
                cmd.motion.translation = this->translation;
            }
            cmd.motion.angle = atan2(msg->axes.at(0), msg->axes.at(1)) + M_PI;
            cmd.motion.rotation = msg->axes.at(3) * this->rotation;
        }

        // cross left/right => increase/decrease rotation
        if (msg->axes.at(6) < 0)
        {
            emit changeRotation(-this->rotationIncrease);
        }
        else if (msg->axes.at(6) > 0)
        {
            emit changeRotation(this->rotationIncrease);
        }

        // cross up/down => increase/decrease translation
        if (msg->axes.at(7) < 0)
        {
            emit changeTranslation(-this->translationIncrease);
        }
        else if (msg->axes.at(7) > 0)
        {
            emit changeTranslation(this->translationIncrease);
        }

        // ballHandle stuff LT/RT => rotate ball handle
        if (this->dribbleManually)
        {
            if (this->ltPressedOnce && this->rtPressedOnce)
            {
                cmd.ballHandleLeftMotor = this->ballHandleSign * ((msg->axes.at(2) - 1) / 2) * this->ballHandleMax;
                cmd.ballHandleRightMotor = this->ballHandleSign * ((msg->axes.at(5) - 1) / 2) * this->ballHandleMax;
            }
        }
        else
        {
            cmd.ballHandleLeftMotor = this->ballHandleLeftMotor;
            cmd.ballHandleRightMotor = this->ballHandleRightMotor;
        }
        if (this->useBallHandle)
        {
            cmd.ballHandleState = msl_msgs::JoystickCommand::BALL_HANDLE_ON;
        }
        else
        {
            cmd.ballHandleState = msl_msgs::JoystickCommand::BALL_HANDLE_OFF;
        }
        cmd.kickPower = this->kickPower;
        cmd.shovelIdx = this->shovelIdx;
        cmd.ptControllerState = this->usePTController;
        cmd.ballHandleState = this->useBallHandle;
        printJoystickMessage(cmd);
        this->joycmd = cmd;
        this->joyPub.publish(cmd);
        emit startJoyTimer();
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
        cout << "PTController State:\t" << (this->usePTController ? "On" : "Off") << endl;
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
        cout << "PTController State:\t" << (this->usePTController ? "On" : "Off") << endl;
        cout << "BallHandle Left:\t" << (int)msg.ballHandleLeftMotor << "\t Right: " << (int)msg.ballHandleRightMotor
                << endl;
    }

} // namespace msl_keyboard_joystick

PLUGINLIB_EXPORT_CLASS(msl_joystick::Joystick, rqt_gui_cpp::Plugin)
