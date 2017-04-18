using namespace std;
#include "Plans/Behaviours/CalcCalib.h"

/*PROTECTED REGION ID(inccpp1446033324019) ENABLED START*/ // Add additional includes here
ros::Publisher calibCoeff_pub;
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <SystemConfig.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1446033324019) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
CalcCalib::CalcCalib()
    : DomainBehaviour("CalcCalib")
{
    /*PROTECTED REGION ID(con1446033324019) ENABLED START*/ // Add additional options here
    calibOldPosMotionX = 0;
    calibOldPosMotionY = 0;
    oldCalibCoefficientX = 0;
    oldCalibCoefficientY = 0;
    correctedPosX = 0;
    correctedPosY = 0;
    calibCounter = 0;
    /*PROTECTED REGION END*/
}
CalcCalib::~CalcCalib()
{
    /*PROTECTED REGION ID(dcon1446033324019) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void CalcCalib::run(void *msg)
{
    /*PROTECTED REGION ID(run1446033324019) ENABLED START*/ // Add additional options here
    auto ownMotionPos = this->wm->rawSensorData->getOwnPositionMotionBuffer().getLastValidContent();
    if (ownMotionPos)
    {
        calibPosMotionX = ownMotionPos->x;
        calibPosMotionY = ownMotionPos->y;
    }

    double theta = this->wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent()->theta;
    correctedWayX = (calibPosMotionX - calibOldPosMotionX) * cos(theta - theta) -
                    (calibPosMotionY - calibOldPosMotionY) * sin(theta - theta);
    correctedWayY = (calibPosMotionX - calibOldPosMotionX) * sin(theta - theta) +
                    (calibPosMotionY - calibOldPosMotionY) * cos(theta - theta);

    correctedPosX = correctedPosX + correctedWayX;
    correctedPosY = correctedPosY + correctedWayY;

    lengthSegment = lengthSegment + sqrt((correctedWayX) * (correctedWayX) + (correctedWayY) * (correctedWayY));

    calibOldPosMotionX = calibPosMotionX;
    calibOldPosMotionY = calibPosMotionY;

    /*PROTECTED REGION END*/
}
void CalcCalib::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1446033324019) ENABLED START*/ // Add additional options here
    auto ownPos = this->wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
    if (ownPos)
    {
        diffX = abs(correctedPosX - ownPos->x);
        diffY = abs(correctedPosY - ownPos->y);
    }
    else
    {
        std::cerr << "CalcCalib: No position available during \"initialiseParameters\"" << std::endl;
    }

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
    calibCoeff_pub = calibCEP.advertise<CalibrationCoefficient>("CalibrationCoefficient", 1);

    if (calibCounter >= 1)
    {
        // mit Mittelwert
        if (calibCounter == 1)
        {
            if (oldCalibCoefficientX > 0)
            {
                oldCalibCoefficientX =
                    (oldCalibCoefficientX + calibSign(ownPos->x, correctedPosX) * (diffX / lengthSegment) + 1) / 2;
                calibCoefficientX *= oldCalibCoefficientX;
            }
            else
            {
                oldCalibCoefficientX = calibSign(ownPos->x, correctedPosX) * (diffX / lengthSegment) + 1;
            }
        }

        if (calibCounter == 2)
        {
            if (oldCalibCoefficientX > 0)
            {
                oldCalibCoefficientX =
                    (oldCalibCoefficientX + calibSign(ownPos->x, correctedPosX) * (diffX / lengthSegment) + 1) / 2;
                calibCoefficientX *= oldCalibCoefficientX;
            }
            else
            {
                oldCalibCoefficientX = calibSign(correctedPosX, ownPos->x) * (diffX / lengthSegment) + 1;
            }
        }

        if (calibCounter == 3)
        {
            if (oldCalibCoefficientY > 0)
            {
                oldCalibCoefficientY =
                    (oldCalibCoefficientY + calibSign(ownPos->y, correctedPosY) * (diffY / lengthSegment) + 1) / 2;
                calibCoefficientY *= oldCalibCoefficientY;
            }
            else
            {
                oldCalibCoefficientY = calibSign(ownPos->y, correctedPosY) * (diffY / lengthSegment) + 1;
            }
        }

        if (calibCounter == 4)
        {
            if (oldCalibCoefficientY > 0)
            {
                oldCalibCoefficientY =
                    (oldCalibCoefficientY + calibSign(correctedPosY, ownPos->y) * (diffY / lengthSegment) + 1) / 2;
                calibCoefficientY *= oldCalibCoefficientY;
            }

            else
            {
                oldCalibCoefficientY = calibSign(correctedPosY, ownPos->y) * (diffY / lengthSegment) + 1;
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

    switch (calibCounter)
    {
    case 1:
        std::cout << "Difference X: " << diffX << " (" << (diffX / lengthSegment) * 100 << " %)\n" << std::endl;
        break;

    case 2:
        std::cout << "Difference X: " << diffX << " (" << (diffX / lengthSegment) * 100 << " %)" << std::endl;
        std::cout << "new calibration coefficient X: " << calibCoefficientX << "\n" << std::endl;
        break;

    case 3:
        std::cout << "Difference Y: " << diffY << " (" << (diffY / lengthSegment) * 100 << " %)\n" << std::endl;
        break;

    case 4:
        std::cout << "Difference Y: " << diffY << " (" << (diffY / lengthSegment) * 100 << " %)" << std::endl;
        std::cout << "new calibration coefficient Y: " << calibCoefficientY << "\n" << std::endl;
        break;

    default:
        std::cout << "\nold calibration coefficient X: " << calibCoefficientX
                  << "\nold calibration coefficient Y: " << calibCoefficientY << "\n"
                  << std::endl;
    }
    /* std::cout << "Differenzen: " << std::endl;
     std::cout << "X: " << diffX << std::endl;
     std::cout << "Y: " << diffY << std::endl;
     std::cout << "FaktorX: " << calibCoefficientX << std::endl;
     std::cout << "FaktorY: " << calibCoefficientY << std::endl;

     std::cout << "" << std::endl;*/

    lengthSegment = 0;
    calibCounter++;
    correctedPosX = ownPos->x;
    correctedPosY = ownPos->y;

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1446033324019) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
