using namespace std;
#include "Plans/TestPlans/DribbleControlTest/DribbleControlMOS.h"

/*PROTECTED REGION ID(inccpp1479905178049) ENABLED START*/ // Add additional includes here
#include <math.h>

#include <SystemConfig.h>

#include <GeometryCalculator.h>

#include <Ball.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>

#include <msl_actuator_msgs/BallHandleCmd.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1479905178049) ENABLED START*/ // initialise static variables here
    /*PROTECTED REGION END*/
    DribbleControlMOS::DribbleControlMOS() :
            DomainBehaviour("DribbleControlMOS")
    {
        /*PROTECTED REGION ID(con1479905178049) ENABLED START*/ // Add additional options here
        this->decayFactor = 0.5;
        this->testingMode = false;
        this->wheelSpeedLeftOld = 0;
        this->wheelSpeedRightOld = 0;

        this->speedNoBall = 0.0;

        this->translationOld = 0.0;
        this->rotationOld = 0.0;
        this->angleOld = 0.0;

        // to increase initial actuator speed
        this->powerFactor = 0.0;
        this->decayedPowerFactor = 0.0;
        this->transTolerance = 0.0;
        this->rotTolerance = 0.0;
        this->angleTolerance = 0.0;
        this->jumpPlanned = false;

        this->staticUpperBound = 0.0;
        this->staticMiddleBound = 0.0;
        this->staticLowerBound = 0.0;
        this->staticNegVelX = 0.0;
        this->epsilonTForward = 0.0;
        this->epsilonTBackward = 0.0;
        this->epsilonY = 0.0;
        this->epsilonRot = 0.0;
        this->rBallRobot = 0.0;
        this->forwConst = 0.0;
        this->sidewConst = 0.0;
        this->diagConst = 0.0;
        this->phi = 0.0;
        this->velYFactor = 0.0;
        this->velXFactor = 0.0;
        this->powerOfRotation = 0.0;
        /*PROTECTED REGION END*/
    }
    DribbleControlMOS::~DribbleControlMOS()
    {
        /*PROTECTED REGION ID(dcon1479905178049) ENABLED START*/ // Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleControlMOS::run(void* msg)
    {
        /*PROTECTED REGION ID(run1479905178049) ENABLED START*/ // Add additional options here
        // if joystick sends own ball handle commands -> return
        shared_ptr < msl_msgs::JoystickCommand > joyCmd = wm->rawSensorData->getJoystickCommand();

        if (joyCmd != nullptr && joyCmd->ballHandleState == msl_msgs::JoystickCommand::BALL_HANDLE_ON)
        {
            return;
        }

        // get odometry data
        shared_ptr < msl_msgs::MotionInfo > odom = nullptr;
        if (this->wm->isUsingSimulator())
        {
            odom = this->wm->rawSensorData->getOwnVelocityVision();
        }
        else
        {
            odom = this->wm->rawSensorData->getOwnVelocityMotion();
        }

        if (odom == nullptr)
        {
            cerr << "DribbleControlMOS: no odometry!" << endl;
            return;
        }

        double robotAngle = odom->angle;
        double robotVel = odom->translation;
        double robotRot = odom->rotation;

        this->jumpPlanned = false;

        // get motion command
        // check if backwards movement is planned
        shared_ptr < msl_actuator_msgs::MotionControl > plannedMotion = this->wm->rawSensorData->getLastMotionCommand();

        if (plannedMotion != nullptr)
        {
            // cout << "DribbleControlMOS::run: planned Motion Angle:" << plannedMotion->motion.angle << endl;

            // if we are not moving at the moment and plan to move backwards
            if (robotVel < 100.0
                    && ((plannedMotion->motion.angle < M_PI / 4.0 && plannedMotion->motion.angle > -M_PI / 4.0)
                            || plannedMotion->motion.angle > M_PI * 7.0 / 4.0)
                    && plannedMotion->motion.translation > 100.0)
            {
                // take planned motion instead of odom values
                robotAngle = plannedMotion->motion.angle;
                robotVel = plannedMotion->motion.translation;
                robotRot = plannedMotion->motion.rotation;
                this->jumpPlanned = true;

                cout << "DribbleControlMOS::run: planned Motion Translation:" << robotVel << endl;
            }
        }

        msl_actuator_msgs::BallHandleCmd msgback;

        if (!this->testingMode && !this->wm->ball->haveBall())
        {
            msgback.rightMotor = -this->speedNoBall;
            msgback.leftMotor = -this->speedNoBall;
            this->sendWheelSpeed(msgback);
            return;
        }

        double velX = 0.0;
        double velY = 0.0;

        // calculates desired ball path depending on robot movement, corrected to guarantee grip
        this->getBallPath(robotVel, robotAngle, robotRot, velX, velY);

        //		auto ballVel = getBallVelocity(velX, velX); <-- maybe bug?
        double ballVel = this->getBallVelocity(velX, velY);
        double ballAngle = this->getBallAngle(velX, velY);
        // cout << "DribbleControlMOS:: ballVel " << ballVel << " ballAngle "<< ballAngle << endl;

        // depends on hardware connection, left and right in this method are as seen from the robots point of view
        msgback.leftMotor = this->getLeftArmVelocity(ballVel, ballAngle);
        msgback.rightMotor = this->getRightArmVelocity(ballVel, ballAngle);
        this->sendWheelSpeed(msgback);

        /*PROTECTED REGION END*/
    }
    void DribbleControlMOS::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1479905178049) ENABLED START*/ // Add additional options her
        this->staticUpperBound = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.staticUpperBound", NULL);
        this->staticMiddleBound = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.staticMiddleBound", NULL);
        this->staticLowerBound = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.staticLowerBound", NULL);
        this->staticNegVelX = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.staticNegVelX", NULL);
        this->rBallRobot = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.rBallRobot", NULL);
        this->epsilonTForward = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.epsilonTForward", NULL);
        this->epsilonTBackward = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.epsilonTBackward", NULL);
        this->epsilonRot = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.epsilonRot", NULL);
        this->epsilonY = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.epsilonY", NULL);
        this->velYFactor = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.velYFactor", NULL);
        this->velXFactor = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.velXFactor", NULL);
        this->powerFactor = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.powerFactor", NULL);
        this->decayFactor = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.decayFactor", NULL);
        this->transTolerance = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.transTolerance", NULL);
        this->rotTolerance = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.rotTolerance", NULL);
        this->angleTolerance = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.angleTolerance", NULL);
        this->testingMode = (*this->sc)["DribbleAlround"]->get<bool>("DribbleAlround.testingMode", NULL);
        this->powerOfRotation = (*this->sc)["DribbleAlround"]->get<double>("DribbleAlround.powerOfRotation", NULL);

        this->speedNoBall = (*this->sc)["Actuation"]->get<double>("Dribble.SpeedNoBall", NULL);

        this->phi = M_PI / 6.0; // horizontal angle between y and arm

        //    this->armInputs = (*this->sc)["Dribble"]->getList<double>("Dribble.ArmVelToInput.InputList", NULL);
        //    this->armLeftVels = (*this->sc)["Dribble"]->getList<double>("Dribble.ArmVelToInput.RightVelList", NULL);
        //    this->armRightVels = (*this->sc)["Dribble"]->getList<double>("Dribble.ArmVelToInput.LeftVelList", NULL);

        // very static
        // forwConst => ~0.89907059794455198110
        this->forwConst = sqrt(
                (sin(M_PI / 2.0 - 0.82)) * sin(M_PI / 2.0 - 0.82)
                        + (sin(0.75) * cos(M_PI / 2.0 - 0.82)) * (sin(0.75) * cos(M_PI / 2.0 - 0.82))) / cos(0.349); // ballVel -> ArmVel for forward

        // sidewConst => ~1.25041800602532376327
        this->sidewConst = sin(0.559) / sin(0.438); // ballVel -> ArmVel for sideways

        // diagConst => ~1.34572549908225510608
        this->diagConst = sin(1.18) / (cos(0.349) * sin(0.82)); // ballVel -> ArmVel for diagonal

        this->wheelSpeedLeftOld = 0.0;
        this->wheelSpeedRightOld = 0.0;

        // TODO SPLINE STUFF
        shared_ptr < vector < string >> speedsSections = (*sc)["Dribble"]->getSections("ArmVelToInput", NULL);
        vector<double> rightArmVel(speedsSections->size());
        vector<double> leftArmVel(speedsSections->size());
        vector<double> inputCurrent(speedsSections->size());
        int i = 0;
        for (string subsection : *speedsSections)
        {
            rightArmVel[i] = (*sc)["Dribble"]->get<double>("ArmVelToInput", subsection.c_str(), "rightVel", NULL);
            leftArmVel[i] = (*sc)["Dribble"]->get<double>("ArmVelToInput", subsection.c_str(), "leftVel", NULL);
            inputCurrent[i] = (*sc)["Dribble"]->get<double>("ArmVelToInput", subsection.c_str(), "input", NULL);
            //            cout << "RobotSpeed: " << robotSpeed[i] << "actuatorSpeed: " << actuatorSpeed[i] << endl;
            i++;
        }
        this->rightSpeedSpline.set_points(rightArmVel, inputCurrent, false);

        this->leftSpeedSpline.set_points(leftArmVel, inputCurrent, false);

        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1479905178049) ENABLED START*/ // Add additional methods here
    double DribbleControlMOS::velToInput(msl::ArmMotor arm, double wheelVelocity)
    {
        double input = 0.0;

        if (wheelVelocity == 0.0)
        {
            return input;
        }

        // model as straight line
        //    double gradient = (3500.0 - 2000.0) / (this->velAt3500 - this->velAt2000);
        //    double input = gradient * (abs(wheelVelocity) - this->velAt2000) + 2000.0;
        //    input *= geometry::sgn(wheelVelocity);

        // model as spline
        if (arm == msl::ArmMotor::LEFT)
        {
            input = max(-10000.0, min(10000.0, leftSpeedSpline(wheelVelocity)));
        }
        else if (arm == msl::ArmMotor::RIGHT)
        {
            input = max(-10000.0, min(10000.0, rightSpeedSpline(wheelVelocity)));
        }
        // cout<<"DribbleControlMOS::velToInput::wheelVelocity "<<wheelVelocity<<" to Input: "<<input<<endl;
        return input;
    }

    void DribbleControlMOS::sendWheelSpeed(msl_actuator_msgs::BallHandleCmd &msgback)
    {
        //    double maxDelta = 100.0;
        //        if (this->wheelSpeedLeftOld < -2000 && this->wheelSpeedRightOld < -2000
        //                && msgback.leftMotor > this->wheelSpeedLeftOld && msgback.rightMotor > this->wheelSpeedRightOld
        //                && fabs(msgback.leftMotor - this->wheelSpeedLeftOld) > maxDelta
        //                && fabs(msgback.rightMotor - this->wheelSpeedRightOld) > maxDelta)
        //        {
        //            msgback.leftMotor = wheelSpeedLeftOld + maxDelta;
        //            msgback.rightMotor = wheelSpeedRightOld + maxDelta;
        //        }
        this->wheelSpeedLeftOld = msgback.leftMotor;
        this->wheelSpeedRightOld = msgback.rightMotor;
        //
        //        msgback.rightMotor = std::abs(msgback.rightMotor) < 100.0 ? 0 : msgback.rightMotor;
        //        msgback.leftMotor = std::abs(msgback.leftMotor) < 100.0 ? 0 : msgback.leftMotor;

        send(msgback);
    }
    /**
     * calculates desired ball path depending on the robot movement and corrected to ensure grib
     */
    void DribbleControlMOS::getBallPath(double translation, double angle, double rotation, double &velX, double &velY)
    {
        // ball velocity form only the robots translation
        double velXTemp = -cos(angle) * translation;
        double velYTemp = -sin(angle) * translation;

        // correcting desired ball velocity towards robot to guarantee grib
        //			velX -= epsilonT * abs(translation) + epsilonRot * abs(rotation);
        velX = velXTemp;

        // correction of velocity in x , depending on x (epsilonT), depending on y (epsilonY)
        // epsilonT<1 ; epsilonY<0.45*velYFactor
        // velX = velX - (epsilonT * velXTemp * sign(velXTemp)) - epsilonY * velYTemp * sign(velYTemp);

        double epsilonT = velXTemp > 0.0 ? this->epsilonTForward : this->epsilonTBackward;
        velX = velX - epsilonT * std::abs(velXTemp) - this->epsilonY * std::abs(velYTemp);

        // correction of velocity in x depending on rotation (epsilonRot)
        if (fabs(velYTemp) > 200.0)
        {
            velX = velX - this->epsilonRot * geometry::sgn(velYTemp) * this->rBallRobot * rotation;
        }
        else
        {
            // rotation goes in nonlinear to fit for high as well as low
            velX = velX
                    - this->epsilonRot * pow(rotation * geometry::sgn(rotation), this->powerOfRotation) * rBallRobot;
        }

        // special case where y velocity and rotation result in positive x velocity (sign(velY)!=sign(rot))
        // TODO

        // rotation results in y velocity of the ball
        velY = velYTemp + 3.0 / 4.0 * this->rBallRobot * rotation;
        // factor so robot can hold the ball if driving sideways
        velY = velY * this->velYFactor;
        velX = velX * this->velXFactor;

        //	double powerFactor = 2;
        //	double transTolerance = 200;
        //	double rotTolerance = 0.4;
        //	double angleTolerance = 0.4;

        // for higher grip when starting motion, we multiply the velocity with powerFactor for the first iterations
        // only for negative x, so we don't push the ball out
        if (velXTemp < 0.0)
        {
            // detect jump in odometry values
            if (this->jumpPlanned || this->transTolerance <= fabs(translation - this->translationOld)
                    || this->rotTolerance <= fabs(rotation - this->rotationOld))
            // || angleTolerance <= fabs(angle - angleOld))
            {
                // powerFactor decays over the iterations
                cout << "DribbleControlMOS::getBallPath: Jump detected" << fabs(translation - this->translationOld)
                        << " " << fabs(rotation - rotationOld) << " " << fabs(angle - this->angleOld) << endl;
                this->decayedPowerFactor = this->powerFactor;
            }

            // velY = velY * (1 + decayedPowerFactor);
            velX = velX * (1.0 + this->decayedPowerFactor);
            this->decayedPowerFactor *= this->decayFactor;
        }

        this->translationOld = translation;
        this->rotationOld = rotation;
        this->angleOld = angle;

        // if we start moving forward, we don't want to push directly
        // x between staticMiddle and staticUpper don't push forward
        if (velXTemp <= this->staticUpperBound && velXTemp >= this->staticMiddleBound
                && velYTemp <= this->staticUpperBound && velYTemp >= staticLowerBound && rotation <= 0.1
                && rotation >= -0.1)
        {
            // results in 0 arm wheel movement
            velX = 0.0;
        }
        // x between staticLower and staticMiddle already pull
        else if (velXTemp <= this->staticMiddleBound && velXTemp >= this->staticLowerBound
                && velYTemp <= this->staticUpperBound && velYTemp >= staticLowerBound && rotation <= 0.1
                && rotation >= -0.1)
        {
            // results in minimum negative arm wheel movement
            velX = -100.0;
        }
    }

    double DribbleControlMOS::getBallVelocity(double velX, double velY)
    {
        return sqrt(velX * velX + velY * velY);
    }

    /**
     * returns values [pi,-pi[
     */
    double DribbleControlMOS::getBallAngle(double velX, double velY)
    {
        return atan2(velY, velX);
    }

    double DribbleControlMOS::getLeftArmVelocity(double ballVelocity, double ballAngle)
    {
        double sec0 = -M_PI;
        double sec1 = -M_PI / 2.0 - this->phi;
        double sec2 = -M_PI / 2.0;
        double sec3 = -M_PI / 2.0 + this->phi;
        double sec4 = 0.0;
        double sec5 = M_PI / 2.0 - this->phi;
        double sec6 = M_PI / 2.0;
        double sec7 = M_PI / 2.0 + this->phi;
        double sec8 = M_PI;

        double angleConst = 0.0;

        // linear interpolation of the constants in the 8 sectors

        if (ballAngle <= sec1)
        {
            // rotate right
            angleConst = -this->forwConst + (ballAngle - sec0) * (-this->diagConst + this->forwConst) / (sec1 - sec0);
        }
        else if (ballAngle <= sec2)
        {
            angleConst = -this->diagConst + (ballAngle - sec1) * (-this->sidewConst + this->diagConst) / (sec2 - sec1);
        }
        else if (ballAngle <= sec3)
        {
            angleConst = -this->sidewConst + (ballAngle - sec2) * (this->sidewConst) / (sec3 - sec2);
        }
        else if (ballAngle <= sec4)
        {
            angleConst = (ballAngle - sec3) * this->forwConst / (sec4 - sec3);
        }
        else if (ballAngle <= sec5)
        {
            angleConst = this->forwConst + (ballAngle - sec4) * (this->diagConst - this->forwConst) / (sec5 - sec4);
        }
        else if (ballAngle <= sec6)
        {
            angleConst = this->diagConst + (ballAngle - sec5) * (this->sidewConst - this->diagConst) / (sec6 - sec5);
        }
        else if (ballAngle <= sec7)
        {
            angleConst = this->sidewConst + (ballAngle - sec6) * (-this->sidewConst) / (sec7 - sec6);
        }
        else if (ballAngle >= sec7)
        {
            // rotate left
            angleConst = (ballAngle - sec7) * (-this->forwConst) / (sec8 - sec7);
        }

        return velToInput(msl::ArmMotor::LEFT, ballVelocity * angleConst);
    }

    double DribbleControlMOS::getRightArmVelocity(double ballVelocity, double ballAngle)
    {

        double sec0 = -M_PI;
        double sec1 = -M_PI / 2.0 - this->phi;
        double sec2 = -M_PI / 2.0;
        double sec3 = -M_PI / 2.0 + this->phi;
        double sec4 = 0.0;
        double sec5 = M_PI / 2.0 - this->phi;
        double sec6 = M_PI / 2.0;
        double sec7 = M_PI / 2.0 + this->phi;
        double sec8 = M_PI;

        double angleConst = 0.0;

        // linear interpolation of the constants in the 8 sectors

        if (ballAngle <= sec1)
        {
            // pull
            angleConst = (ballAngle - sec1) * this->forwConst / (sec1 - sec0);
        }
        // wheel stops
        else if (ballAngle <= sec2)
        {
            // push
            angleConst = (ballAngle - sec1) * this->sidewConst / (sec2 - sec1);
        }
        else if (ballAngle <= sec3)
        {

            angleConst = this->sidewConst + (ballAngle - sec2) * (this->diagConst - this->sidewConst) / (sec3 - sec2);
        }
        else if (ballAngle <= sec4)
        {
            angleConst = this->diagConst + (ballAngle - sec3) * (this->forwConst - this->diagConst) / (sec4 - sec3);
        }
        else if (ballAngle <= sec5)
        {
            // push
            angleConst = this->forwConst + (ballAngle - sec4) * (-this->forwConst) / (sec5 - sec4);
        }
        // wheel stops
        else if (ballAngle <= sec6)
        {
            // pull
            angleConst = (ballAngle - sec5) * (-this->sidewConst) / (sec6 - sec5);
        }
        else if (ballAngle <= sec7)
        {
            angleConst = -this->sidewConst + (ballAngle - sec6) * (this->sidewConst - this->diagConst) / (sec7 - sec6);
        }
        else if (ballAngle >= sec7)
        {
            // rotate left
            angleConst = -this->diagConst + (ballAngle - sec7) * (-this->forwConst + this->diagConst) / (sec8 - sec7);
        }

        return velToInput(msl::ArmMotor::RIGHT, ballVelocity * angleConst);
    }

/*PROTECTED REGION END*/
} /* namespace alica */
