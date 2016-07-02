using namespace std;
#include "Plans/Dribble/DribbleControl.h"

/*PROTECTED REGION ID(inccpp1449742071382) ENABLED START*/ //Add additional includes here
#include <RawSensorData.h>
#include <Ball.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <SystemConfig.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1449742071382) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DribbleControl::DribbleControl() :
            DomainBehaviour("DribbleControl")
    {
        /*PROTECTED REGION ID(con1449742071382) ENABLED START*/ //Add additional options here
        readConfigParameters();
        /*PROTECTED REGION END*/
    }
    DribbleControl::~DribbleControl()
    {
        /*PROTECTED REGION ID(dcon1449742071382) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleControl::run(void* msg)
    {
        /*PROTECTED REGION ID(run1449742071382) ENABLED START*/ //Add additional options here
        msl_actuator_msgs::BallHandleCmd bhc;
	cout << "in DribbleControl!!!!!!" << endl;
        // for anticipated pass perception, testing, or what you like
        if (pullNoMatterWhat)
        {
            bhc.leftMotor = (int)-max(-10000.0, min(10000.0, speedNoBall));
            bhc.rightMotor = (int)-max(-10000.0, min(10000.0, speedNoBall));

            //If we are close to the ball give more speed
//            shared_ptr < geometry::CNPoint2D > b = wm->ball.getEgoBallPosition();
//            if (b != nullptr && b->length() < 400)
//            {
//                bhc.leftMotor = max(bhc.leftMotor, (int) - 80);
//                bhc.rightMotor = max(bhc.rightMotor, (int) - 80);
//            }

            send(bhc);
            return;
        }

        // get some data and make some checks
        auto motion = wm->rawSensorData->getOwnVelocityMotion();
        shared_ptr < geometry::CNPoint2D > ball = wm->ball->getEgoBallPosition();

        double l = 0;
        double r = 0;
        double orthoL = 0;
        double orthoR = 0;
        double speed = 0;
        if (motion == nullptr)
        {
            return;
        }

        // do we have the ball, so that controlling make sense
        haveBall = wm->ball->haveBall();

        if (haveBall && itcounter++ < 8)
        {
            cout << "DribbleControl: less than 8 iterations have ball" << endl;
            speed = speedNoBall;
        }
        else if (haveBall || controlNoMatterWhat || itcounter >= 8)
        {
            // we have the ball to control it, or want to control ignoring the have ball flag, or we tried to pull it for < X iterations
	    cout << "haveBall = " << haveBall << endl;
            double speedX = cos(motion->angle) * motion->translation;
            double speedY = sin(motion->angle) * motion->translation;
            cout << "DribbleControl: angle:\t" << motion->angle << " trans:\t" << motion->translation << endl;
            cout << "DribbleControl: speedX:\t" << speedX << endl;
            cout << "DribbleControl: speedY:\t" << speedY << endl;

            //geschwindigkeitsanteil fuer rotation nur beachten, falls rotation größer bzw kleiner 1/-1
            double rotation = motion->rotation;

            if (rotation < 0)
            {
                l = 0;
                r = fabs(rotation) * curveRotationFactor / M_PI;
            }
            else
            {
                r = 0;
                l = fabs(rotation) * curveRotationFactor / M_PI;
            }

            //ignor rotation error
            if (fabs(rotation) < 0.04)
            {
                l = 0;
                r = 0;
            }

            //langsam vorwaerts
            if (speedX > -slowTranslation && speedX < 40)
            {
                speed = slowTranslationWheelSpeed;
            }
            //langsam rueckwaerts
            else if (speedX < slowTranslation && speedX >= 40)
            {
                speed = -slowTranslationWheelSpeed;
            }
            //schnell vor
            else if (speedX <= -slowTranslation)
            {
                //0.5 is for correct rounding
                speed = max(-10000.0, min(10000.0, forwardSpeedSpline(speedX) + 0.5));
		speed = speed * (1-rotation / 4.0);
                // if (rotation > -1.0 && rotation < 1.0 && speedX <= -800)
//                if (rotation > -1.0 && rotation < 1.0)
//                {
//                    l = 0;
//                    r = 0;
//                }
            }
            //schnell rueck
            else
            {
                speed = max(-10000.0, min(10000.0, 3 * handlerSpeedFactor * speedX / 100.0));
            }

            //geschwindigkeitsanteil fuer orthogonal zum ball
            if (speedY > 0)
            {
                //nach rechts fahren
                orthoR = speedY * orthoDriveFactor;
                orthoL = -speedY * orthoDriveFactor / 3.0; // replaced with higher Denominator ... was 2
            }
            else
            {
                //nach links fahren
                orthoR = speedY * orthoDriveFactor / 3.0; // replaced with higher Denominator .. was 2
                orthoL = -speedY * orthoDriveFactor;
            }
        }
        else if (!haveBall)
        {
            // we don't have the ball
            speed = speedNoBall;
        }

        cout << "DribbleControl: Left: speed: \t" << speed << " orthoL: \t" << orthoL << " l: \t" << l << endl;
        cout << "DribbleControl: Right: speed: \t" << speed << " orthoR: \t" << orthoR << " r: \t" << r << endl;
        bhc.leftMotor = (int)-max(-10000.0, min(10000.0, speed + l + orthoL));
        bhc.rightMotor = (int)-max(-10000.0, min(10000.0, speed + r + orthoR));

        hadBefore = haveBall;
        if (!hadBefore)
        {
            cout << "DribbleControl: Reset Counter" << endl;
            itcounter = 0;
        }

        send(bhc);
        /*PROTECTED REGION END*/
    }
    void DribbleControl::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1449742071382) ENABLED START*/ //Add additional options here
        this->hadBefore = false;
        string tmp;
        bool success = true;
        try
        {
            success &= getParameter("ControlNoMatterWhat", tmp);
            if (success)
            {
                std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
                istringstream(tmp) >> std::boolalpha >> controlNoMatterWhat;
            }
            success &= getParameter("PullNoMatterWhat", tmp);
            if (success)
            {
                std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
                istringstream(tmp) >> std::boolalpha >> pullNoMatterWhat;
            }
        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "DC: Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1449742071382) ENABLED START*/ //Add additional methods here
    void DribbleControl::readConfigParameters()
    {
        supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
        handlerSpeedFactor = (*sys)["Actuation"]->get<double>("Dribble.SpeedFactor", NULL);
        speedNoBall = (*sys)["Actuation"]->get<double>("Dribble.SpeedNoBall", NULL);
        handlerSpeedSummand = (*sys)["Actuation"]->get<double>("Dribble.SpeedSummand", NULL);
        slowTranslation = (*sys)["Actuation"]->get<double>("Dribble.SlowTranslation", NULL);
        slowTranslationWheelSpeed = (*sys)["Actuation"]->get<double>("Dribble.SlowTranslationWheelSpeed", NULL);
        curveRotationFactor = (*sys)["Actuation"]->get<double>("Dribble.CurveRotationFactor", NULL);
        orthoDriveFactor = (*sys)["Actuation"]->get<double>("Dribble.OrthoDriveFactor", NULL);

        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

        shared_ptr < vector<string> > speedsSections = (*sc)["Actuation"]->getSections("ForwardDribbleSpeeds", NULL);
        vector<double> robotSpeed(speedsSections->size());
        vector<double> actuatorSpeed(speedsSections->size());
        int i = 0;
        for (string subsection : *speedsSections)
        {
            robotSpeed[i] = (*sc)["Actuation"]->get<double>("ForwardDribbleSpeeds", subsection.c_str(), "robotSpeed",
                                                            NULL);
            actuatorSpeed[i] = (*sc)["Actuation"]->get<double>("ForwardDribbleSpeeds", subsection.c_str(),
                                                               "actuatorSpeed", NULL);
            cout << "RobotSpeed: " << robotSpeed[i] << "actuatorSpeed: " << actuatorSpeed[i] << endl;
            i++;
        }
        forwardSpeedSpline.set_points(robotSpeed, actuatorSpeed, false);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
