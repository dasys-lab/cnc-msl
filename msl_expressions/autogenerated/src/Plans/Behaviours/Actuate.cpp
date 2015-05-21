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
        auto rodo = wm->rawSensorData.getOwnVelocityMotion();
        int left, right;
        // TODO x und y wahrscheinlich durch merge verloren gegangen, nochmal anschauen
        double x, y;

        if (rodo == nullptr)
        {
            cout << "Actuate RODO is empty help" << endl;
            return;
        }

        //QualityOfService= wm->rawSensorData.getOpticalFlow()->qos;
        // x Werte richtig vertauscht

        //Mittelwert der Aktuellen 4 Werte

        double arithmeticAverage = 0;
        double newParamer = wm->rawSensorData.getOwnVelocityMotion()->translation;
        list<double>::iterator parameter;

        if (arithmeticAverageBox.size() == 4)
        {
            arithmeticAverageBox.pop_back();
        }

        arithmeticAverageBox.push_front(newParamer);

        for (parameter = arithmeticAverageBox.begin(); parameter != arithmeticAverageBox.end(); parameter++)
        {
            arithmeticAverage += *parameter;
        }

        arithmeticAverage = arithmeticAverage / 8;

        //Vorsteuerung
        //VorneRechts
        if (wm->rawSensorData.getOwnVelocityMotion()->angle <= M_PI
                && wm->rawSensorData.getOwnVelocityMotion()->angle >= M_PI / 2)

            x = pow(cos(wm->rawSensorData.getOwnVelocityMotion()->angle), 2);
        y = pow(sin(wm->rawSensorData.getOwnVelocityMotion()->angle), 2);

        left = arithmeticAverage * (x + y) / 10;
        right = arithmeticAverage * (x - y) / 10;

        //HintenRechts
        if (wm->rawSensorData.getOwnVelocityMotion()->angle > 0
                && wm->rawSensorData.getOwnVelocityMotion()->angle < M_PI / 2)

            x = pow(cos(wm->rawSensorData.getOwnVelocityMotion()->angle), 2);
        y = pow(sin(wm->rawSensorData.getOwnVelocityMotion()->angle), 2);

        left = arithmeticAverage * (x + y) / 10;
        right = arithmeticAverage * (x - y) / 10;

        if (wm->rawSensorData.getOwnVelocityMotion()->angle < 0)
        {
            wm->rawSensorData.getOwnVelocityMotion()->angle = wm->rawSensorData.getOwnVelocityMotion()->angle + M_PI;

            //HintenLinks
            if (wm->rawSensorData.getOwnVelocityMotion()->angle >= M_PI / 2
                    && wm->rawSensorData.getOwnVelocityMotion()->angle <= M_PI)

                y = -pow(cos(wm->rawSensorData.getOwnVelocityMotion()->angle), 2);
            x = -pow(sin(wm->rawSensorData.getOwnVelocityMotion()->angle), 2);

            left = arithmeticAverage * (x - y) / 10;
            right = arithmeticAverage * (x + y) / 10;

            //VorneLinks
            if (wm->rawSensorData.getOwnVelocityMotion()->angle >= M_PI / 2
                    && wm->rawSensorData.getOwnVelocityMotion()->angle <= M_PI)

                y = pow(cos(wm->rawSensorData.getOwnVelocityMotion()->angle), 2);
            x = pow(sin(wm->rawSensorData.getOwnVelocityMotion()->angle), 2);

            left = arithmeticAverage * (x - y) / 10;
            right = arithmeticAverage * (y + x) / 10;

        }
        //left=arithmeticAverage;
        //right=arithmeticAverage;

        // left =  rodo->motion.translation * (1.0 / 40.0) ;
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
