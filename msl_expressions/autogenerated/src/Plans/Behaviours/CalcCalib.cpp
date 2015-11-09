using namespace std;
#include "Plans/Behaviours/CalcCalib.h"
double posMotionY;
double posMotionX;
/*PROTECTED REGION ID(inccpp1446033324019) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1446033324019) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CalcCalib::CalcCalib() :
            DomainBehaviour("CalcCalib")
    {
        /*PROTECTED REGION ID(con1446033324019) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    CalcCalib::~CalcCalib()
    {
        /*PROTECTED REGION ID(dcon1446033324019) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CalcCalib::run(void* msg)
    {
        /*PROTECTED REGION ID(run1446033324019) ENABLED START*/ //Add additional options here

    	if(this->wm->rawSensorData.getOwnPositionMotion(0)->x!= posMotionX || this->wm->rawSensorData.getOwnPositionMotion(0)->y!= posMotionY)
    	{
    		if (this->wm->rawSensorData.getOwnPositionVision(0) != NULL)
    		{
    			auto posVision = this->wm->rawSensorData.getOwnPositionVision(0);
    			auto oldPosVision = this->wm->rawSensorData.getOwnPositionVision(1);
    		}
            auto posMotion = this->wm->rawSensorData.getOwnPositionMotion(0);
            posMotionX = this->wm->rawSensorData.getOwnPositionMotion(0)->x;
            posMotionY = this->wm->rawSensorData.getOwnPositionMotion(0)->y;
            auto oldPosMotion = this->wm->rawSensorData.getOwnPositionMotion(1);

            this->wm->calibData.length = this->wm->calibData.length
                    + sqrt((posMotion->x - oldPosMotion->x) * (posMotion->x - oldPosMotion->x)
                            + (posMotion->y - oldPosMotion->y) * (posMotion->y - oldPosMotion->y));
            std::cout << "posMotion: "<< this->wm->rawSensorData.getOwnPositionMotion(0)->x - this->wm->rawSensorData.getOwnPositionMotion(1)->x<< std::endl;
    	}
        /*PROTECTED REGION END*/
    }
    void CalcCalib::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1446033324019) ENABLED START*/ //Add additional options here
        if (this->wm->rawSensorData.getOwnPositionVision(0) != NULL)
        {
            auto deltax = this->wm->rawSensorData.getOwnPositionMotion(0)->x
                    - this->wm->rawSensorData.getOwnPositionVision(0)->x;
            auto deltay = this->wm->rawSensorData.getOwnPositionMotion(0)->y
                    - this->wm->rawSensorData.getOwnPositionVision(0)->y;

            if (this->wm->calibData.length != 0)
            {
                this->wm->calibData.calibCoefficient = (sqrt(deltax * deltax + deltay * deltay)
                        / this->wm->calibData.length) + 1;
                string filename = string(sc->getConfigPath()) + string(sc->getHostname()) + string("/CalibData.txt");
                ofstream saveToCalibData;
                saveToCalibData.open(filename);
                saveToCalibData << this->wm->calibData.calibCoefficient;
                saveToCalibData.close();
            }
            std::cout << "X: " << this->wm->rawSensorData.getOwnPositionVision(0)->x << std::endl;
            std::cout << "Y: " << this->wm->rawSensorData.getOwnPositionVision(0)->y << std::endl;
            std::cout << "Differenzen: " << std::endl;
            std::cout << "X: " << deltax << std::endl;
            std::cout << "Y: " << deltay << std::endl;
            std::cout << "Länge: " << this->wm->calibData.length << std::endl;
            std::cout << "Faktor: " << this->wm->calibData.calibCoefficient << std::endl;
            std::cout << "posMotion: "<< this->wm->rawSensorData.getOwnPositionMotion(0)->x<< std::endl;
            std::cout << "oldposMotion: "<< this->wm->rawSensorData.getOwnPositionMotion(1)->x<< std::endl;
            std::cout << "posVision: "<< this->wm->rawSensorData.getOwnPositionVision(0)->x<< std::endl;
            std::cout << "oldposVision: "<< this->wm->rawSensorData.getOwnPositionVision(1)->x<< std::endl;
            std::cout << "" << std::endl;
        }

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1446033324019) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
