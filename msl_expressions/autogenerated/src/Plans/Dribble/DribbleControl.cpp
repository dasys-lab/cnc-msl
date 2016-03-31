using namespace std;
#include "Plans/Dribble/DribbleControl.h"

/*PROTECTED REGION ID(inccpp1449742071382) ENABLED START*/ //Add additional includes here
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

        // for anticipated pass perception, testing, or what you like
        if (pullNoMatterWhat)
        {
            bhc.leftMotor = (int8_t) - max(-100.0, min(100.0, speedNoBall));
            bhc.rightMotor = (int8_t) - max(-100.0, min(100.0, speedNoBall));

            //If we are close to the ball give more speed
//            shared_ptr < geometry::CNPoint2D > b = wm->ball.getEgoBallPosition();
//            if (b != nullptr && b->length() < 400)
//            {
//                bhc.leftMotor = max(bhc.leftMotor, (int8_t) - 80);
//                bhc.rightMotor = max(bhc.rightMotor, (int8_t) - 80);
//            }

            send(bhc);
            return;
        }

        // get some data and make some checks
        auto motion = wm->rawSensorData.getOwnVelocityMotion();
        shared_ptr < geometry::CNPoint2D > ball = wm->ball.getEgoBallPosition();

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
        haveBall = wm->ball.haveBall();

//        if (haveBall && !hadBefore)
  //      {
//		cout << "DribbleControl: Reset Counter" << endl;
  //          itcounter = 0;
        //}

        if (haveBall && itcounter++ < 8)
        {
		cout << "DribbleControl: less than 8 iterations have ball" << endl;
            speed = speedNoBall;
        }
        else if (true || haveBall || controlNoMatterWhat || itcounter >= 8)
        {
            // we have the ball to control it, or want to control ignoring the have ball flag, or we tried to pull it for < X iterations


            double speedX = cos(motion->angle) * motion->translation;
            double speedY = sin(motion->angle) * motion->translation;
	cout << "DribbleControl: angle:\t" << motion->angle << " trans:\t" << motion->translation << endl;
//	cout << "DribbleControl: speedX:\t" << speedX << endl;
//	cout << "DribbleControl: speedY:\t" << speedY << endl;

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
                speed = max(-100.0, min(100.0, handlerSpeedFactor * speedX / 100.0));
            }
            //schnell rueck
            else
            {
                speed = max(-100.0, min(100.0, 3 * handlerSpeedFactor * speedX / 100.0));
            }

            //geschwindigkeitsanteil fuer orthogonal zum ball
            if (speedY > 0)
            {
                //nach rechts fahren
                orthoR = speedY * orthoDriveFactor;
                orthoL = -speedY * orthoDriveFactor / 2.0;
            }
            else
            {
                //nach links fahren
                orthoR = speedY * orthoDriveFactor / 2.0;
                orthoL = -speedY * orthoDriveFactor;
            }

            //geschwindigkeitsanteil fuer rotation
            double rotation = motion->rotation;
            if (rotation < 0)
            {
                l = 0;
                r = abs(rotation) / M_PI * curveRotationFactor;
            }
            else
            {
                r = 0;
                l = abs(rotation) / M_PI * curveRotationFactor;
            }

        }
        else if (!haveBall)
        {
            // we don't have the ball
            speed = speedNoBall;
        }

	
	cout << "DribbleControl: Left: speed: \t" << speed << " orthoL: \t" << orthoL << " l: \t" << l << endl;
	cout << "DribbleControl: Right: speed: \t" << speed << " orthoR: \t" << orthoR << " r: \t" << r << endl;
        bhc.leftMotor = (int8_t) - max(-100.0, min(100.0, speed + l + orthoL));
        bhc.rightMotor = (int8_t) - max(-100.0, min(100.0, speed + r + orthoR));

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
        slowTranslation = (*sys)["Actuation"]->get<double>("Dribble.SlowTranslation", NULL);
        slowTranslationWheelSpeed = (*sys)["Actuation"]->get<double>("Dribble.SlowTranslationWheelSpeed", NULL);
        curveRotationFactor = (*sys)["Actuation"]->get<double>("Dribble.CurveRotationFactor", NULL);
        orthoDriveFactor = (*sys)["Actuation"]->get<double>("Dribble.OrthoDriveFactor", NULL);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
