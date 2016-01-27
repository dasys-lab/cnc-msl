using namespace std;
#include "Plans/Behaviours/CalcCalib.h"


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

        if (this->wm->rawSensorData.getOwnPositionVision(0) != NULL)
        {
            calibPosVisionX = this->wm->rawSensorData.getOwnPositionVision(0)->x;
            calibPosVisionY = this->wm->rawSensorData.getOwnPositionVision(0)->y;
            }

        calibPosMotionX = this->wm->rawSensorData.getOwnPositionMotion(0)->x;
        calibPosMotionY = this->wm->rawSensorData.getOwnPositionMotion(0)->y;

        correctedWayX = (calibPosMotionX - calibOldPosMotionX)
                / cos(this->wm->rawSensorData.getOwnPositionVision(0)->theta
                        - this->wm->rawSensorData.getOwnPositionMotion(0)->theta);
        correctedWayY = (calibPosMotionY - calibOldPosMotionY)
                * tan(this->wm->rawSensorData.getOwnPositionVision(0)->theta
                        - this->wm->rawSensorData.getOwnPositionMotion(0)->theta);

        correctedPosX = correctedPosX + correctedWayX;
        correctedPosY = correctedPosY + correctedWayY;

        lengthSegment = lengthSegment + sqrt((correctedWayX) * (correctedWayX) + (correctedWayY) * (correctedWayY));
        length = length + sqrt((correctedWayX) * (correctedWayX) + (correctedWayY) * (correctedWayY));

        calibOldPosMotionX = calibPosMotionX;
        calibOldPosMotionY = calibPosMotionY;

        if(tempyoyo == 10)
        {
        	static int visionLengthCounter;
        	lengthVision = lengthVision + sqrt((calibOldPosVisionX - calibPosVisionX) * (calibOldPosVisionX - calibPosVisionX) + (calibOldPosVisionY - calibPosVisionY) * (calibOldPosVisionY - calibPosVisionY));

            calibOldPosVisionX = calibPosVisionX;
            calibOldPosVisionY = calibPosVisionY;
            tempyoyo = 0;
            visionLengthCounter++;
            std::cout << "Counter: " << visionLengthCounter <<std::endl;

        }

        tempyoyo++;


        //msl_actuator_msgs::MotionControl mc;
        //mc.motion.translation = 500;
        //send(mc);

        //}
        /*PROTECTED REGION END*/
    }
    void CalcCalib::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1446033324019) ENABLED START*/ //Add additional options here

            diffX = correctedPosX - this->wm->rawSensorData.getOwnPositionVision(0)->x;
            diffY = correctedPosY - this->wm->rawSensorData.getOwnPositionVision(0)->y;

            string value;
            string filename = string(sc->getConfigPath()) + string(sc->getHostname()) + string("/CalibData.txt");
            ifstream calibData(filename);
            if (calibData.is_open())
            {
                while (getline(calibData, value))
                {
                    calibCoefficient = std::stod(value);
                }
                calibData.close();
            }

            if (calibCoefficient == 0)
            {
                calibCoefficient = 1;
            }

            if (length != 0)
            {
                if (length > 12500) //GonzalesUpdate
                {

                   calibCoefficient *= calibSign(lengthVision, length) * (sqrt(diffX * diffX + diffY * diffY)
                            / lengthSegment) + 1; //GonzalesUpdate + lengthSegment

                }

                string filename = string(sc->getConfigPath()) + string(sc->getHostname()) + string("/CalibData.txt");
                ofstream saveToCalibData;
                saveToCalibData.open(filename);
                saveToCalibData << calibCoefficient;
                saveToCalibData.close();

            }

            std::cout << "Differenzen: " << std::endl;
            std::cout << "X: " << diffX << std::endl;
            std::cout << "Y: " << diffY << std::endl;
            std::cout << "Länge: " << length << std::endl;
            std::cout << "Faktor: " << calibCoefficient << std::endl;
            std::cout << "posMotionX: " << this->wm->rawSensorData.getOwnPositionMotion(0)->x << std::endl;
            std::cout << "posMotionY: " << this->wm->rawSensorData.getOwnPositionMotion(0)->y << std::endl;
            std::cout << "correctedWayX : " << correctedPosX << std::endl;
            std::cout << "correctedWayY : " << correctedPosY << std::endl;
            std::cout << "posVisionX: " << this->wm->rawSensorData.getOwnPositionVision(0)->x << std::endl;
            std::cout << "posVisionY: " << this->wm->rawSensorData.getOwnPositionVision(0)->y << std::endl;
            std::cout << "Faktor2 : "
                    << calibSign(lengthVision, length) * (sqrt(diffX * diffX + diffY * diffY)
                            / length) + 1 << std::endl;
            std::cout << "oldPosMotionX: " << calibOldPosMotionX << std::endl;
            std::cout << "posMotionX: " << calibPosMotionX << std::endl;

            std::cout<< "lengthSegment: " << lengthSegment <<std::endl;
            std::cout<< "lengthVision: " << lengthVision <<std::endl;

            std::cout << "" << std::endl;

            lengthSegment = 0;
            correctedPosX = this->wm->rawSensorData.getOwnPositionVision(0)->x;
            correctedPosY = this->wm->rawSensorData.getOwnPositionVision(0)->y;

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1446033324019) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
