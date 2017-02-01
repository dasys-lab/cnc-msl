using namespace std;
#include "Plans/Behaviours/CalcCalib.h"

/*PROTECTED REGION ID(inccpp1446033324019) ENABLED START*/ //Add additional includes here
ros::Publisher calibCoeff_pub;
#include <RawSensorData.h>
#include <MSLWorldModel.h>
#include <SystemConfig.h>
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
        calibPosMotionX = this->wm->rawSensorData->getOwnPositionMotion()->x;
        calibPosMotionY = this->wm->rawSensorData->getOwnPositionMotion()->y;

        correctedWayX = (calibPosMotionX - calibOldPosMotionX)
                * cos(this->wm->rawSensorData->getOwnPositionVision()->theta
                        - this->wm->rawSensorData->getOwnPositionMotion()->theta)
                - (calibPosMotionY - calibOldPosMotionY)
                        * sin(this->wm->rawSensorData->getOwnPositionVision()->theta
                                - this->wm->rawSensorData->getOwnPositionMotion()->theta);
        correctedWayY = (calibPosMotionX - calibOldPosMotionX)
                * sin(this->wm->rawSensorData->getOwnPositionVision()->theta
                        - this->wm->rawSensorData->getOwnPositionMotion()->theta)
                + (calibPosMotionY - calibOldPosMotionY)
                        * cos(this->wm->rawSensorData->getOwnPositionVision()->theta
                                - this->wm->rawSensorData->getOwnPositionMotion()->theta);

        correctedPosX = correctedPosX + correctedWayX;
        correctedPosY = correctedPosY + correctedWayY;

        lengthSegment = lengthSegment + sqrt((correctedWayX) * (correctedWayX) + (correctedWayY) * (correctedWayY));

        calibOldPosMotionX = calibPosMotionX;
        calibOldPosMotionY = calibPosMotionY;

        /*PROTECTED REGION END*/
    }
    void CalcCalib::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1446033324019) ENABLED START*/ //Add additional options here
        diffX = abs(correctedPosX - this->wm->rawSensorData->getOwnPositionVision()->x);
        diffY = abs(correctedPosY - this->wm->rawSensorData->getOwnPositionVision()->y);

        string value;
        string filename = string(sc->getConfigPath()) + string(sc->getHostname()) + string("/CalibData.txt");
        ifstream calibData(filename);
        if (calibData.is_open())
        {
            for (int lineno = 0; getline(calibData, value) && lineno < 2; lineno++)
            {
                if (lineno == 0)
                {

                    calibCoefficientX = std::stod(value);
                }

                if (lineno == 1)
                {
                    calibCoefficientY = std::stod(value);
                }
            }
            calibData.close();
        }
        else
        {
            std::cout << "keine CalibData!" << std::endl;
        }

        ros::NodeHandle calibCEP;
        calibCoeff_pub = calibCEP.advertise < CalibrationCoefficient > ("CalibrationCoefficient", 1);

        if (calibCounter >= 1)
        {
            // mit Mittelwert
            if (calibCounter == 1)
            {
                if (oldCalibCoefficientX > 0)
                {
                    oldCalibCoefficientX = (oldCalibCoefficientX
                            + calibSign(this->wm->rawSensorData->getOwnPositionVision()->x, correctedPosX)
                                    * (diffX / lengthSegment) + 1) / 2;
                    calibCoefficientX *= oldCalibCoefficientX;
                }
                else
                {
                    oldCalibCoefficientX = calibSign(this->wm->rawSensorData->getOwnPositionVision()->x, correctedPosX)
                            * (diffX / lengthSegment) + 1;
                }

            }

            if (calibCounter == 2)
            {
                if (oldCalibCoefficientX > 0)
                {
                    oldCalibCoefficientX = (oldCalibCoefficientX
                            + calibSign(correctedPosX, this->wm->rawSensorData->getOwnPositionVision()->x)
                                    * (diffX / lengthSegment) + 1) / 2;
                    calibCoefficientX *= oldCalibCoefficientX;
                }
                else
                {
                    oldCalibCoefficientX = calibSign(correctedPosX, this->wm->rawSensorData->getOwnPositionVision()->x)
                            * (diffX / lengthSegment) + 1;
                }

            }

            if (calibCounter == 3)
            {
                if (oldCalibCoefficientY > 0)
                {
                    oldCalibCoefficientY = (oldCalibCoefficientY
                            + calibSign(this->wm->rawSensorData->getOwnPositionVision()->y, correctedPosY)
                                    * (diffY / lengthSegment) + 1) / 2;
                    calibCoefficientY *= oldCalibCoefficientY;
                }
                else
                {
                    oldCalibCoefficientY = calibSign(this->wm->rawSensorData->getOwnPositionVision()->y, correctedPosY)
                            * (diffY / lengthSegment) + 1;
                }

            }

            if (calibCounter == 4)
            {
                if (oldCalibCoefficientY > 0)
                {
                    oldCalibCoefficientY = (oldCalibCoefficientY
                            + calibSign(correctedPosY, this->wm->rawSensorData->getOwnPositionVision()->y)
                                    * (diffY / lengthSegment) + 1) / 2;
                    calibCoefficientY *= oldCalibCoefficientY;
                }

                else
                {
                    oldCalibCoefficientY = calibSign(correctedPosY, this->wm->rawSensorData->getOwnPositionVision()->y)
                            * (diffY / lengthSegment) + 1;
                }

            }
        }
        // mit Mittelwert Ende

        // ohne Mittelwert
        /*
         //if (abs(correctedPosX - oldCorrectedPosX) > 500){
         //if (correctedPosX > oldCorrectedPosX){
         if(calibCounter == 1){
         calibCoefficientX *= calibSign(this->wm->rawSensorData->getOwnPositionVision()->x, correctedPosX)
         * (diffX / lengthSegment) + 1;

         }

         //if (correctedPosX < oldCorrectedPosX){
         if(calibCounter == 2){
         calibCoefficientX *= calibSign(correctedPosX, this->wm->rawSensorData->getOwnPositionVision()->x)
         * (diffX / lengthSegment) + 1;
         }
         //}

         //if (abs(correctedPosY - oldCorrectedPosY) > 300){
         //if (correctedPosY > oldCorrectedPosY){
         if(calibCounter == 3){
         calibCoefficientY *= calibSign(this->wm->rawSensorData->getOwnPositionVision()->y, correctedPosY)
         * (diffY / lengthSegment) + 1;

         }
         //if (correctedPosY < oldCorrectedPosY){
         if(calibCounter == 4){
         calibCoefficientY *= calibSign(correctedPosY, this->wm->rawSensorData->getOwnPositionVision()->y)
         * (diffY / lengthSegment) + 1;

         }
         //}
         */
        // Ohne Mittelwert Ende
        if (calibCoefficientX < 0.3)
        {
            calibCoefficientX = 0.3;
        }
        if (calibCoefficientY < 0.3)
        {
            calibCoefficientY = 0.3;
        }

        if (calibCoefficientX > 2)
        {
            calibCoefficientX = 2;
        }

        if (calibCoefficientY > 2)
        {
            calibCoefficientY = 2;
        }

        ofstream saveToCalibData;
        saveToCalibData.open(filename);
        saveToCalibData << calibCoefficientX << "\n";
        saveToCalibData << calibCoefficientY;
        saveToCalibData.close();

        calibCoeff.calibCoefficientX = calibCoefficientX;
        calibCoeff.calibCoefficientY = calibCoefficientY;
        calibCoeff_pub.publish(calibCoeff);

        std::cout << "Differenzen: " << std::endl;
        std::cout << "X: " << diffX << std::endl;
        std::cout << "Y: " << diffY << std::endl;
        std::cout << "FaktorX: " << calibCoefficientX << std::endl;
        std::cout << "FaktorY: " << calibCoefficientY << std::endl;
        std::cout << "posMotionX: " << this->wm->rawSensorData->getOwnPositionMotion()->x << std::endl;
        std::cout << "posMotionY: " << this->wm->rawSensorData->getOwnPositionMotion()->y << std::endl;
        std::cout << "correctedPosX : " << correctedPosX << std::endl;
        std::cout << "correctedPosY : " << correctedPosY << std::endl;
        std::cout << "posVisionX: " << this->wm->rawSensorData->getOwnPositionVision()->x << std::endl;
        std::cout << "posVisionY: " << this->wm->rawSensorData->getOwnPositionVision()->y << std::endl;
        std::cout << "lengthSegment: " << lengthSegment << std::endl;
        std::cout << "oldCoeffX: " << oldCalibCoefficientX << std::endl;
        std::cout << "oldCoeffY: " << oldCalibCoefficientY << std::endl;
        std::cout << "calibCounter: " << calibCounter << std::endl;

        std::cout << "" << std::endl;

        lengthSegment = 0;
        calibCounter++;
        correctedPosX = this->wm->rawSensorData->getOwnPositionVision()->x;
        correctedPosY = this->wm->rawSensorData->getOwnPositionVision()->y;

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1446033324019) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
