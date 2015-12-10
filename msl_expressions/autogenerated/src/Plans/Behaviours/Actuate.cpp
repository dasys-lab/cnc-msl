using namespace std;
#include "Plans/Behaviours/Actuate.h"

/*PROTECTED REGION ID(inccpp1417017518918) ENABLED START*/ //Add additional includes here
#include "math.h"
#include "RawSensorData.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <CubicSplineInterpolation/Spline.h>

/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1417017518918) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Actuate::Actuate() :
            DomainBehaviour("Actuate")
    {
        /*PROTECTED REGION ID(con1417017518918) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    Actuate::~Actuate()
    {
        /*PROTECTED REGION ID(dcon1417017518918) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Actuate::run(void* msg)
    {
        /*PROTECTED REGION ID(run1417017518918) ENABLED START*/ //Add additional options here
        msl_actuator_msgs::BallHandleCmd bhc;
        auto rodo = wm->rawSensorData.getOwnVelocityMotion();

        double left = 0.0;
        double right = 0.0;

        if (rodo == nullptr)
        {
            cout << "Actuate RODO is empty help" << endl;
            return;
        }

        newController(left, right);

        //PD Regler Anfang
        //PIDControllerLeft
        
         const double Ki =(*this->sc)["ActuatorDribble"]->get<double>("ActuateDribble.KiLeft", NULL);
         const double Kd =(*this->sc)["ActuatorDribble"]->get<double>("ActuateDribble.KdLeft", NULL);
         const double Kp = (*this->sc)["ActuatorDribble"]->get<double>("ActuateDribble.KpLeft", NULL);
         
         double AbweichungLeft = 0.0;
         double Abweichung_SummeLeft = 0.0;
         double Abweichung_AltLeft = 0.0;
         double StellwertLeft = 0.0; 
       	 double Sollwert;
	 const double setPointLeft = (*this->sc)["ActuatorDribble"]->get<double>("ActuateDribble.setPointLeft", NULL);
        
         
	if(cos(wm->rawSensorData.getOwnVelocityMotion()->angle)<0)
	{
	Sollwert=25.0;
	}
	
	if(cos(wm->rawSensorData.getOwnVelocityMotion()->angle)>0)
        {
        Sollwert=70;
        }

	
         if ((wm->rawSensorData.getOpticalFlowQoS() <= Sollwert)&&(wm->rawSensorData.getOwnVelocityMotion()->translation>500))
        {
         
         Abweichung_SummeLeft += AbweichungLeft;
         AbweichungLeft = -1 * (setPointLeft - wm->rawSensorData.getOpticalFlowQoS());
         StellwertLeft = Kp * AbweichungLeft +left;
         StellwertLeft += Ki * Abweichung_SummeLeft;
         StellwertLeft += Kd * (AbweichungLeft - Abweichung_AltLeft);
         
         Abweichung_AltLeft = AbweichungLeft;
         left=StellwertLeft;

	 };
         
         //PIDControllerRight
         
       
         
         const double setPointRight =(*this->sc)["ActuatorDribble"]->get<double>("ActuateDribble.setPointRight", NULL);
         
         
         double AbweichungRight = 0.0;
         double Abweichung_SummeRight = 0.0;
         double Abweichung_AltRight = 0.0;
         double StellwertRight = 0.0;
         
         if ((wm->rawSensorData.getOpticalFlowQoS() <= Sollwert)&&(wm->rawSensorData.getOwnVelocityMotion()->translation>500))
         {
         Abweichung_SummeRight += AbweichungRight;
         AbweichungRight = -1 * (setPointRight - wm->rawSensorData.getOpticalFlowQoS());
         StellwertRight = Kp * AbweichungRight + right;
         StellwertRight += Ki * Abweichung_SummeRight;
         StellwertRight += Kd * (AbweichungRight - Abweichung_AltRight);
         
         Abweichung_AltRight = AbweichungRight;
         right=StellwertRight;
	
	 };
         
   
         
         //PD Regler Ende
        
	cout<<"left:  "<<left<<"    Righ: "<<right<<endl;
        cout << "Winkel : " << wm->rawSensorData.getOwnVelocityMotion()->angle << endl;
        cout << "QualityOfService WM : " << wm->rawSensorData.getOpticalFlowQoS() << endl;
        cout << " rotation : " << wm->rawSensorData.getOwnVelocityMotion()->rotation << endl;

        bhc.leftMotor = max(min(left, 60.0), -100.0);
        bhc.rightMotor = max(min(right, 60.0), -100.0);
        this->send(bhc);

        speedDifferenceNew = wm->rawSensorData.getOwnVelocityMotion()->translation;

        /*PROTECTED REGION END*/
    }
    void Actuate::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1417017518918) ENABLED START*/ //Add additional options here
        speedDifference = 0.0;
        double zaeler = 0;
        double qualityOfServiceSumme = 0;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1417017518918) ENABLED START*/ //Add additional methods here
    //////////OldController Start
    void Actuate::oldController(double &leftController, double &rightController)
    {

        bool pullNoMatterWhat = false;
        bool controlNoMatterWhat = false;
        bool haveBall = false;
        int itcounter = 0;
        double handlerSpeedFactor = 1.6;
        double speedNoBall = 40;
        double slowTranslation = 100;
        double slowTranslationWheelSpeed = 15;
        double curveRotationFactor = 80;
        double orthoDriveFactor = 0.09;
        double maxhundred = 100;

        double MaxPWM = 90;

        double UpPos = 0;
        double CatchingPos = 100;

        double PassingPos = 28;
        double NormalPos = 90;

        double PingInterval = 200;
        double ShovelSelectRepeatInterval = 40;
        double HaveLightBarrier = 0;

        //speed factor of both wheels dependent on the odometry
        double SpeedFactor = 1.6;

        //speed of wheels if we dont have the ball
        double SpeedNoBall = 40;
        //slow translation of robot, and the speed of the wheels
        double SlowTranslation = 100;

        double SlowTranslationWheelSpeed = 15;
        double CurveRotationFactor = 80;
        double BackwardsSpeed = 52;
        double OrthoDriveFactor = 0.09;

        double LinearFactor = 1.0;
        double UseFactor = false;

        SpeedNoBall = 50;
        SlowTranslation = 200;
        double SlowRotationLeft = 10;
        double SlowRotationRight = 10;

        double RotationLeft = -38;
        double RotationRight = 38;

        BackwardsSpeed = 52;

        double kp = 0.01;
        double ki = 0.01;
        double kd = 0.00008;
        double MinQos = 50;
        double VelocityFactor = 25;
        double VelocityDiff = 0;
        double l = 0, r = 0;
        double orthoL = 0, orthoR = 0;
        double speed = 0;

        // do we have the ball, so that controlling make sense
        /*		haveBall = WorldHelper.HaveBallDribble(WM, WorldHelper.HadBallBefore);

         if (haveBall && !hadBefore)
         {
         itcounter = 0;
         }
         */
        //		if (haveBall && itcounter++ < 8)
        //		{
        //			speed = speedNoBall;
        //		}
        if (true)
        {
            // we have the ball to control it, or want to control ignoring the have ball flag, or we tried to pull it for < X iterations

            double speedX = cos(wm->rawSensorData.getOwnVelocityMotion()->angle)
                    * wm->rawSensorData.getOwnVelocityMotion()->translation;
            double speedY = sin(wm->rawSensorData.getOwnVelocityMotion()->angle)
                    * wm->rawSensorData.getOwnVelocityMotion()->translation;
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
                double minSpeedOne = handlerSpeedFactor * speedX / 100.0;
                speed = max(-maxhundred, min(maxhundred, minSpeedOne));
            }
            //schnell rueck
            else
            {
                double minSpeedTwo = 3 * handlerSpeedFactor * speedX / 100.0;
                speed = max(-maxhundred, min(maxhundred, minSpeedTwo));
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
            double rotation = wm->rawSensorData.getOwnVelocityMotion()->rotation;
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

        double minSpeedThreeLeft = speed + l + orthoL;
        double minSpeedThreeRight = speed + r + orthoR;
        double KvLeft;
        double KvRight;

        KvLeft = -1.0 * max(-maxhundred, min(maxhundred, minSpeedThreeLeft));
        KvRight = -1.0 * max(-maxhundred, min(maxhundred, minSpeedThreeRight));

        cout << "OldController " << endl;
        cout << "KvRight : " << KvRight << endl;
        cout << "KvLeft : " << KvLeft << endl;

        leftController = KvLeft;
        rightController = KvRight;
        //////////OldController End
    }

    void Actuate::newController(double &leftController, double &rightController)
    {
        ///////////////////////////////////////////////////////////////////////
        //New Controller Start

        cout << "NewController " << endl;

        //arithmetic Average for Speed Start

        cout << "aritAverageSpeed anfang" << endl;

        double arithmeticAverageSpeed = 0.0;
        double newParamerSpeed = wm->rawSensorData.getOwnVelocityMotion()->translation;
        //double wtf = wm->rawSensorData.getLastMotionCommand()->motion;

        if (arithmeticAverageBoxSpeed.size() == 2)
        {
            arithmeticAverageBoxSpeed.pop_back();
        }

        arithmeticAverageBoxSpeed.push_front(newParamerSpeed);

        for (list<double>::iterator parameterSpeed = arithmeticAverageBoxSpeed.begin();
                parameterSpeed != arithmeticAverageBoxSpeed.end(); parameterSpeed++)
        {
            arithmeticAverageSpeed += *parameterSpeed;
        }

        arithmeticAverageSpeed = arithmeticAverageSpeed / 2;

        cout<<" Speed Approx : " << arithmeticAverageSpeed << " <=> real "
                << wm->rawSensorData.getOwnVelocityMotion()->translation << endl;

        //arithmetic Average for Speed End

        cout << "Z:324 speed diff 4 acceleration" << endl;
        
        //Speed Difference for acceleration Start
        double eFunktionAcceleration;
        double newSpeed = wm->rawSensorData.getOwnVelocityMotion()->translation;
        speedDifference = newSpeed - speedDifferenceNew;
        speedDifference = (speedDifference) / 300;

        if (speedDifference < 1)
        {
            speedDifference = 1;
        }

        cout << "Z:336 speedDifference : " << speedDifference << endl;

        //Speed Difference for acceleration End

        ////arithmetic average speed difference Start

        cout << "arithmetic average speed difference Start";
        double arithmeticAverageSpeedDifference = 0.0;

        if (arithmeticAverageBoxSpeedDifference.size() == 5)
        {
            arithmeticAverageBoxSpeedDifference.pop_back();
        }

        arithmeticAverageBoxSpeedDifference.push_front(speedDifference);

        for (list<double>::iterator parameterSpeedDifference = arithmeticAverageBoxSpeedDifference.begin();
                parameterSpeedDifference != arithmeticAverageBoxSpeedDifference.end(); parameterSpeedDifference++)
        {
            arithmeticAverageSpeedDifference = arithmeticAverageSpeedDifference + *parameterSpeedDifference;
        }

        arithmeticAverageSpeedDifference = arithmeticAverageSpeedDifference / 5;

        if (arithmeticAverageSpeedDifference < 1)
        {
            arithmeticAverageSpeedDifference = 1;
        }

        //Exp Funktion for traction Start
        cout << "Z:366 Exp Funktion for traction Start" << endl;

        double feedForwardLeft, feedForwardRight;
        double KvLeft, KvRight;
        double a,t;
        double angle = wm->rawSensorData.getOwnVelocityMotion()->angle;

        double valueExpFunktion = (*this->sc)["ActuatorDribble"]->get<double>("ActuateDribble.valueExpFunktion", NULL);
        double constPushUpFunktion = (*this->sc)["ActuatorDribble"]->get<double>("ActuateDribble.constPushUpFunktion",
                                                                                 NULL);
        double funktionLeft = 0, funktionRight = 0;
        cout << "Z:377 Exp Funktion for traction end" << endl;
        //für fehlersuche ausk. double qualityOfService = wm->rawSensorData.getOpticalFlowQoS();
        // double eFunktion = valueExpFunktion * (0.0184 + 0.039637 * exp(-0.003 * arithmeticAverageSpeed));

        // cout << "exp Funktion : " << eFunktion << endl;
        //Exp Funktion for traction End
        //Funktion for drive with differt angles start
        double x = angle; // max(min(angle, 3.14), -3.14);
        int counter = 1;
        cout << "vor den Splines left" << endl;
        //Exp Funktion for traction End
        //Funktion for drive with differt angles start
        a = max(min(angle, 3.14), -3.14);
        t=wm->rawSensorData.getOwnVelocityMotion()->translation;
        counter = 1;

        splines::spline leftMotor;

        vector<double> XLeft, YLeft;
        auto FunktionValuesLeftSections = (*this->sc)["ActuatorDribble"]->getSections(
                "ActuateDribble.FunktionValuesLeft", NULL);
        counter = 1;
        for (string sectionName : *FunktionValuesLeftSections)
        {
            XLeft.push_back(
                    (*this->sc)["ActuatorDribble"]->get<double>("ActuateDribble.FunktionValuesLeft",
                                                                sectionName.c_str(), "XLeft", NULL));
            YLeft.push_back(
                    (*this->sc)["ActuatorDribble"]->get<double>("ActuateDribble.FunktionValuesLeft",
                                                                sectionName.c_str(), "YLeft", NULL));
            counter++;
        }
        leftMotor.set_points(XLeft, YLeft);

        cout << "vor der splines right" << endl;
        splines::spline rightMotor;

        vector<double> XRight, YRight;
        auto FunktionValuesRightSections = (*this->sc)["ActuatorDribble"]->getSections(
                "ActuateDribble.FunktionValuesRight", NULL);
        counter = 1;
        for (string sectionName : *FunktionValuesRightSections)
        {
            XRight.push_back(
                    (*this->sc)["ActuatorDribble"]->get<double>("ActuateDribble.FunktionValuesRight",
                                                                sectionName.c_str(), "XRight", NULL));
            YRight.push_back(
                    (*this->sc)["ActuatorDribble"]->get<double>("ActuateDribble.FunktionValuesRight",
                                                                sectionName.c_str(), "YRight", NULL));
            counter++;
        }

        cout << "Z: 420 vor rightMotorset" << endl;

        rightMotor.set_points(XRight, YRight);

        cout << "Z: 424 ende der splines right" << endl;

        rightMotor.set_points(XRight, YRight);


        splines::spline frictionSpline;
        vector<double> XFric, YFric;
        for (int i = 1; i <= 4; i++)
        {
            XFric.push_back(
                    (*this->sc)["ActuatorDribble"]->get<double>(
                            (std::string("ActuateDribble.Friction.x") + to_string(i)).c_str(), NULL));
            YFric.push_back(
                    (*this->sc)["ActuatorDribble"]->get<double>(
                            (std::string("ActuateDribble.Friction.y") + to_string(i)).c_str(), NULL));
        }

//<<<<<<< HEAD
//        cout << "Z: 431 ende der  friction" << endl;
//        frictionSpline.set_points(XFric, YFric);
//        cout << "Z: 431 ende der  friction 1" << endl;
//
//        FunktionValuesRight = rightMotor(x);
//        cout << "Z: 431 ende der  friction 2" << endl;
//
//        FunktionValuesLeft = leftMotor(x);
//        cout << "Z: 431 ende der  friction 3" << endl;
//=======

        frictionSpline.set_points(XFric, YFric);

        FunktionValuesRight = rightMotor(a);
        FunktionValuesLeft = leftMotor(a);
        frictionValue = frictionSpline(t);

        cout << "FunktionValuesRight: "<<FunktionValuesRight<<endl<<"FunktionValuesLeft: "<<FunktionValuesLeft<<endl;
        cout<<"FrictionValue: " << frictionValue<<endl ;


        double rotationLeft = 0.0;
        double rotationRight = 0.0;

        if (((wm->rawSensorData.getOwnVelocityMotion()->rotation > 0.25)&&(wm->rawSensorData.getOwnVelocityMotion()->translation<700))||((wm->rawSensorData.getOwnVelocityMotion()->rotation > 0.8)&&(wm->rawSensorData.getOwnVelocityMotion()->translation>700)))
        {

            rotationLeft = -wm->rawSensorData.getOwnVelocityMotion()->rotation * 35 - 10;
            rotationRight = -wm->rawSensorData.getOwnVelocityMotion()->rotation * 4 - 5;
            cout << "rotation	left : " << rotationLeft << endl;
            cout << "rotation right : " << rotationRight << endl;
            FunktionValuesLeft = 0;
            if (cos(wm->rawSensorData.getOwnVelocityMotion()->angle) > 0)
            {
                cout << "rueckwaertsdrehen" << endl;
                FunktionValuesRight = FunktionValuesRight + abs(wm->rawSensorData.getOwnVelocityMotion()->rotation);
            }
            if (cos(wm->rawSensorData.getOwnVelocityMotion()->angle) < 0)
            {

                FunktionValuesLeft = FunktionValuesLeft + abs(wm->rawSensorData.getOwnVelocityMotion()->rotation) * 0.5;

            }
        }

          if (((wm->rawSensorData.getOwnVelocityMotion()->rotation < -0.25)&&(wm->rawSensorData.getOwnVelocityMotion()->translation<700))||((wm->rawSensorData.getOwnVelocityMotion()->rotation <-0.8)&&(wm->rawSensorData.getOwnVelocityMotion()->translation>700)))

        {

            rotationRight = wm->rawSensorData.getOwnVelocityMotion()->rotation * 35 - 10;
            rotationLeft = wm->rawSensorData.getOwnVelocityMotion()->rotation * 4 - 5;

            cout << "rotation	left : " << rotationLeft << endl;
            cout << "rotation right : " << rotationRight << endl;

            FunktionValuesRight = 0;

            if (cos(wm->rawSensorData.getOwnVelocityMotion()->angle) > 0)
            {

                cout << "rückwärtsdrehen" << endl;
                FunktionValuesLeft = FunktionValuesLeft + abs(wm->rawSensorData.getOwnVelocityMotion()->rotation);

            }
            if (cos(wm->rawSensorData.getOwnVelocityMotion()->angle) < 0)
            {

                FunktionValuesRight = FunktionValuesRight
                        + abs(wm->rawSensorData.getOwnVelocityMotion()->rotation) * 0.5;

            }

        }

        //Rotation Controller End

        KvRight = (0.9 * frictionValue * FunktionValuesRight + rotationRight);
        KvLeft = (0.9 * frictionValue * FunktionValuesLeft + rotationLeft);

        
        cout << "KvLeft : " << KvLeft << endl;
        cout << "KvRight : " << KvRight << endl;

        //Funktion for drive with differt angles end
        leftController = KvLeft;
        rightController = KvRight;

        //Sensor calibration Start

        double teiler;
        qualityOfServiceSumme = qualityOfServiceSumme + wm->rawSensorData.getOpticalFlowQoS();
        zaeler = zaeler + 1;
        teiler = qualityOfServiceSumme / zaeler;

        //	cout << "qualityOfServiceSumme  : " << qualityOfServiceSumme << endl;
        //	cout << "zaeler  : " << zaeler << endl;
        //	cout << "teiler  : " << teiler << endl;

        //Sensor calibration End

    }

/*PROTECTED REGION END*/
} /* namespace alica */
