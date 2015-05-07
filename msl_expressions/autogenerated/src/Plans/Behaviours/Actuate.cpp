using namespace std;
#include "Plans/Behaviours/Actuate.h"

/*PROTECTED REGION ID(inccpp1417017518918) ENABLED START*/ //Add additional includes here
#include "math.h"
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
        msl_actuator_msgs::RawOdometryInfoPtr rodo = wm->getRawOdometryInfo();
        int left, right;
        // TODO x und y wahrscheinlich durch merge verloren gegangen, nochmal anschauen
        double x,y;

        if (rodo == nullptr)
        {
            cout << "Actuate RODO is empty help" << endl;
            return;
        }

        //QualityOfService= wm->rawSensorData.getOpticalFlow()->qos;
        // x Werte richtig vertauscht
        if ((wm->rawSensorData.getOwnVelocityMotion()->angle <= M_PI / 2)
                && (wm->rawSensorData.getOwnVelocityMotion()->angle >= (-1) * M_PI / 2))
        {
            x = wm->rawSensorData.getOwnVelocityMotion()->translation
                    * pow(cos(wm->rawSensorData.getOwnVelocityMotion()->angle), 2);
        }

        else
        {
            x = (-1) * wm->rawSensorData.getOwnVelocityMotion()->translation
                    * pow(cos(wm->rawSensorData.getOwnVelocityMotion()->angle), 2);
        }
        //y Werte richtig vertauscht
        if (wm->rawSensorData.getOwnVelocityMotion()->angle <= 0)
        {
            y = wm->rawSensorData.getOwnVelocityMotion()->translation
                    * pow(sin(wm->rawSensorData.getOwnVelocityMotion()->angle), 2);
        }
        else
        {
            y = -wm->rawSensorData.getOwnVelocityMotion()->translation
                    * pow(sin(wm->rawSensorData.getOwnVelocityMotion()->angle), 2);
        }
        //Addition der x und y Geschwindigkeitsanteile

        //RoboterD: front left

        if ((y >= 0) && (x >= 0))
        {

            left = (x * 15 + y);
            right = (x * 15 - y); //x-y
        }
        //RoboterD front right
        if ((y < 0) && (x >= 0))
        {

            left = (x * 15 + y);
            right = (x * 15 - y); //x-y
        }

        //RoboterD behind left
        if ((y >= 0) && (x <= 0))
        {

            left = (x - y);
            right = (-x - y);
        }

        //RoboterD behind right
        if ((y <= 0) && (x <= 0))
        {
            left = (x - y);
            right = (x + y);

        }
        // left = rodo->motion.translation * (1.0 / 40.0) ;
        //right = rodo->motion.translation * (1.0 / 40.0) ;

        bhc.leftMotor = max(min(left, 60), -60);
        bhc.rightMotor = max(min(right, 60), -60);

        this->send(bhc);
        /*PROTECTED REGION END*/
    }
    void Actuate::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1417017518918) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1417017518918) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
