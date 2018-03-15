using namespace std;
#include "Plans/TestPlans/DribbleControlTest/DribbleControlMOS.h"

/*PROTECTED REGION ID(inccpp1479905178049) ENABLED START*/ // Add additional includes here
#include "Ball.h"
#include "MSLWorldModel.h"
#include "RawSensorData.h"
#include "SystemConfig.h"
#include "math.h"
#include "msl_actuator_msgs/BallHandleCmd.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1479905178049) ENABLED START*/ // initialise static variables here
    /*PROTECTED REGION END*/
    DribbleControlMOS::DribbleControlMOS() :
            DomainBehaviour("DribbleControlMOS")
    {
        /*PROTECTED REGION ID(con1479905178049) ENABLED START*/ // Add additional options here
        this->sc = supplementary::SystemConfig::getInstance();
        decayFactor = 0.5;
        testingMode = false;
        this->wheelSpeedLeftOld = 0;
        this->wheelSpeedRightOld = 0;

        speedNoBall = 0;

        translationOld = 0;
        rotationOld = 0;
        angleOld = 0;

        // to increase initial actuator speed
        powerFactor = 0;
        decayedPowerFactor = 0;
        transTolerance = 0;
        rotTolerance = 0;
        angleTolerance = 0;

        velToInput = 0;
        staticUpperBound = 0;
        staticMiddleBound = 0;
        staticLowerBound = 0;
        staticNegVelX = 0;
        epsilonTForward = 0;
	epsilonTBackward = 0;
        epsilonY = 0;
        epsilonRot = 0;
        rBallRobot = 0;
        forwConst = 0;
        sidewConst = 0;
        diagConst = 0;
        phi = 0;
        velYFactor = 0;
        velXFactor = 0;
        powerOfRotation = 0;
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
        //if joystick sends own ball handle commands -> return
        auto joyCmd = wm->rawSensorData->getJoystickCommand();

        if (joyCmd != nullptr && joyCmd->ballHandleState == msl_msgs::JoystickCommand::BALL_HANDLE_ON)
        {
            return;
        }

        //get odometry data
        shared_ptr < msl_msgs::MotionInfo > odom = nullptr;
        if (wm->isUsingSimulator())
        {
            odom = wm->rawSensorData->getOwnVelocityVision();
        }
        else
        {
            odom = wm->rawSensorData->getOwnVelocityMotion();
        }

        if (odom == nullptr)
        {
            cerr << "DribbleControlMOS: no odometry!" << endl;
            return;
        }




		auto robotAngle = odom->angle;
        auto robotVel = odom->translation;
        auto robotRot = (double)odom->rotation;

        //get motion command
        //check if backwards movement is planned
        shared_ptr<msl_actuator_msgs::MotionControl> plannedMotion = wm->rawSensorData->getLastMotionCommand();

        cout<<"DribbleControlMOS::run: planned Motion Angle:"<<plannedMotion->motion.angle<<endl;

        //angle query might be wrong at the moment it expects angles between 0 and 2pi
        //if we are not moving at the moment and plan to move backwards
        if (robotVel<50 && (plannedMotion->motion.angle < M_PI/4 || plannedMotion->motion.angle > M_PI*7/4) && plannedMotion->motion.translation > 100) {
        	//take planned motion instead of odom values
        	robotAngle = plannedMotion->motion.angle;
        	robotVel = plannedMotion->motion.translation;
        	robotRot = (double)plannedMotion->motion.rotation;
        }


        msl_actuator_msgs::BallHandleCmd msgback;

        bool haveBall = wm->ball->haveBall();

        if (!testingMode && !haveBall)
        {
            msgback.rightMotor = -speedNoBall;
            msgback.leftMotor = -speedNoBall;
            sendWheelSpeed(msgback);
            return;
        }

        double velX;
        double velY;

        // calculates desired ball path depending on robot movement, corrected to guarantee grip
        getBallPath(robotVel, robotAngle, robotRot, velX, velY);

        //		auto ballVel = getBallVelocity(velX, velX); <-- maybe bug?
        auto ballVel = getBallVelocity(velX, velY);
        auto ballAngle = getBallAngle(velX, velY);

        // calculates dribble wheel velocities depending on Ball path
        auto right = getRightArmVelocity(ballVel, ballAngle);
        auto left = getLeftArmVelocity(ballVel, ballAngle);

        // depends on hardware connection, left and right in this method are as seen from the robots point of view
        msgback.leftMotor = right;
        msgback.rightMotor = left;
        sendWheelSpeed(msgback);

        /*PROTECTED REGION END*/
    }
    void DribbleControlMOS::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1479905178049) ENABLED START*/ // Add additional options her
        velToInput = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.velToInput", NULL);
        staticUpperBound = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.staticUpperBound", NULL);
        staticMiddleBound = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.staticMiddleBound", NULL);
        staticLowerBound = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.staticLowerBound", NULL);
        staticNegVelX = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.staticNegVelX", NULL);
        rBallRobot = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.rBallRobot", NULL);
        epsilonT = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.epsilonT", NULL);
        epsilonRot = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.epsilonRot", NULL);
        epsilonY = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.epsilonY", NULL);
        velYFactor = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.velYFactor", NULL);
        velXFactor = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.velXFactor", NULL);
        powerFactor = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.powerFactor", NULL);
        decayFactor = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.decayFactor", NULL);
        transTolerance = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.transTolerance", NULL);
        rotTolerance = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.rotTolerance", NULL);
        angleTolerance = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.angleTolerance", NULL);
        testingMode = (*sc)["DribbleAlround"]->get<bool>("DribbleAlround.testingMode", NULL);
        powerOfRotation = (*sc)["DribbleAlround"]->get<double>("DribbleAlround.powerOfRotation", NULL);

        speedNoBall = (*sc)["Actuation"]->get<double>("Dribble.SpeedNoBall", NULL);

        phi = M_PI / 6; // horizontal angle between y and arm

        // very static
        // forwConst => ~0.89907059794455198110
        forwConst = sqrt(
                (sin(M_PI / 2 - 0.82)) * sin(M_PI / 2 - 0.82)
                        + (sin(0.75) * cos(M_PI / 2 - 0.82)) * (sin(0.75) * cos(M_PI / 2 - 0.82))) / cos(0.349); // ballVel -> ArmVel for forward

        // sidewConst => ~1.25041800602532376327
        sidewConst = sin(0.559) / sin(0.438); // ballVel -> ArmVel for sideways

        // diagConst => ~1.34572549908225510608
        diagConst = sin(1.18) / (cos(0.349) * sin(0.82)); // ballVel -> ArmVel for diagonal

        this->wheelSpeedLeftOld = 0;
        this->wheelSpeedRightOld = 0;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1479905178049) ENABLED START*/ // Add additional methods here
    void DribbleControlMOS::sendWheelSpeed(msl_actuator_msgs::BallHandleCmd &msgback)
    {
        double maxDelta = 100.0;

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

        msgback.rightMotor = std::abs(msgback.rightMotor) < 100.0 ? 0 : msgback.rightMotor;
        msgback.leftMotor = std::abs(msgback.leftMotor) < 100.0 ? 0 : msgback.leftMotor;

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
        velX = velX - (epsilonT * velXTemp * sign(velXTemp)) - epsilonY * velYTemp * sign(velYTemp);

        // correction of velocity in x depending on rotation (epsilonRot)
        if (fabs(velYTemp) > 200)
        {
            velX = velX - epsilonRot * sign(velYTemp) * rBallRobot * rotation;
        }
        else
        {
            // rotation goes in nonlinear to fit for high as well as low
            velX = velX - epsilonRot * pow(rotation * sign(rotation), powerOfRotation) * rBallRobot;
        }

        // special case where y velocity and rotation result in positive x velocity (sign(velY)!=sign(rot))
        //TODO

        // rotation results in y velocity of the ball
        velY = velYTemp + 3.0 / 4.0 * rBallRobot * rotation;
        // factor so robot can hold the ball if driving sideways
        velY = velY * velYFactor;
        velX = velX * velXFactor;

        //	double powerFactor = 2;
        //	double transTolerance = 200;
        //	double rotTolerance = 0.4;
        //	double angleTolerance = 0.4;

        // for higher grip when starting motion, we multiply the velocity with powerFactor for the first iterations
        // only for negative x, so we don't push the ball out
        if (velXTemp < 0)
        {
            // detect jump in odometry values
            if (transTolerance <= fabs(translation - translationOld) || rotTolerance <= fabs(rotation - rotationOld)
                    || angleTolerance <= fabs(angle - angleOld))
            {
                // powerFactor decays over the iterations
            	cout<<"DribbleControlMOS::getBallPath: Jump detected"<<endl;
                decayedPowerFactor = powerFactor;
            }

            //velY = velY * (1 + decayedPowerFactor);
            velX = velX * (1 + decayedPowerFactor);
            decayedPowerFactor *= decayFactor;

        }

        translationOld = translation;
        rotationOld = rotation;
        angleOld = angle;

        // if we start moving forward, we don't want to push directly
        if (velXTemp <= staticUpperBound && velXTemp >= staticMiddleBound && velYTemp <= staticUpperBound
                && velYTemp >= staticLowerBound && rotation <= 0.1 && rotation >= -0.1)
        {
            // results in 0 arm wheel movement
            velX = 0;
        }
        else if (velXTemp <= staticMiddleBound && velXTemp >= staticLowerBound && velYTemp <= staticUpperBound
                && velYTemp >= staticLowerBound && rotation <= 0.1 && rotation >= -0.1)
        {
            // results in minimum negative arm wheel movement
            velX = -100;
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

    int DribbleControlMOS::sign(double x)
    {
        if (x == 0)
            return x;
        return x > 0 ? 1 : -1;
    }

    double DribbleControlMOS::getLeftArmVelocity(double ballVelocity, double ballAngle)
    {
        double sec0 = -M_PI;
        double sec1 = -M_PI / 2 - phi;
        double sec2 = -M_PI / 2;
        double sec3 = -M_PI / 2 + phi;
        double sec4 = 0;
        double sec5 = M_PI / 2 - phi;
        double sec6 = M_PI / 2;
        double sec7 = M_PI / 2 + phi;
        double sec8 = M_PI;

        double angleConst = 0;

        // linear interpolation of the constants in the 8 sectors

        if (ballAngle <= sec1)
        {
            // pull
            angleConst = (ballAngle - sec1) * forwConst / (sec1 - sec0);
        }
        // wheel stops
        else if (ballAngle <= sec2)
        {
            // push
            angleConst = (ballAngle - sec1) * sidewConst / (sec2 - sec1);
        }
        else if (ballAngle <= sec3)
        {

            angleConst = sidewConst + (ballAngle - sec2) * (diagConst - sidewConst) / (sec3 - sec2);
        }
        else if (ballAngle <= sec4)
        {
            angleConst = diagConst + (ballAngle - sec3) * (forwConst - diagConst) / (sec4 - sec3);
        }
        else if (ballAngle <= sec5)
        {
            // push
            angleConst = forwConst + (ballAngle - sec4) * (-forwConst) / (sec5 - sec4);
        }
        // wheel stops
        else if (ballAngle <= sec6)
        {
            // pull
            angleConst = (ballAngle - sec5) * (-sidewConst) / (sec6 - sec5);
        }
        else if (ballAngle <= sec7)
        {
            angleConst = -sidewConst + (ballAngle - sec6) * (sidewConst - diagConst) / (sec7 - sec6);
        }
        else if (ballAngle >= sec7)
        {
            // rotate left
            angleConst = -diagConst + (ballAngle - sec7) * (-forwConst + diagConst) / (sec8 - sec7);
        }

        return ballVelocity * angleConst * velToInput;
    }

    double DribbleControlMOS::getRightArmVelocity(double ballVelocity, double ballAngle)
    {
        double sec0 = -M_PI;
        double sec1 = -M_PI / 2 - phi;
        double sec2 = -M_PI / 2;
        double sec3 = -M_PI / 2 + phi;
        double sec4 = 0;
        double sec5 = M_PI / 2 - phi;
        double sec6 = M_PI / 2;
        double sec7 = M_PI / 2 + phi;
        double sec8 = M_PI;

        double angleConst = 0;

        // linear interpolation of the constants in the 8 sectors

        if (ballAngle <= sec1)
        {
            // rotate right
            angleConst = -forwConst + (ballAngle - sec0) * (-diagConst + forwConst) / (sec1 - sec0);
        }
        else if (ballAngle <= sec2)
        {
            angleConst = -diagConst + (ballAngle - sec1) * (-sidewConst + diagConst) / (sec2 - sec1);
        }
        else if (ballAngle <= sec3)
        {
            angleConst = -sidewConst + (ballAngle - sec2) * (sidewConst) / (sec3 - sec2);
        }
        else if (ballAngle <= sec4)
        {
            angleConst = (ballAngle - sec3) * forwConst / (sec4 - sec3);
        }
        else if (ballAngle <= sec5)
        {
            angleConst = forwConst + (ballAngle - sec4) * (diagConst - forwConst) / (sec5 - sec4);
        }
        else if (ballAngle <= sec6)
        {
            angleConst = diagConst + (ballAngle - sec5) * (sidewConst - diagConst) / (sec6 - sec5);
        }
        else if (ballAngle <= sec7)
        {
            angleConst = sidewConst + (ballAngle - sec6) * (-sidewConst) / (sec7 - sec6);
        }
        else if (ballAngle >= sec7)
        {
            // rotate left
            angleConst = (ballAngle - sec7) * (-forwConst) / (sec8 - sec7);
        }

        return ballVelocity * angleConst * velToInput;
    }

/*PROTECTED REGION END*/
} /* namespace alica */
