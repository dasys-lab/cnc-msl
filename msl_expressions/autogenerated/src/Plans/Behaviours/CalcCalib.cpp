using namespace std;
#include "Plans/Behaviours/CalcCalib.h"

/*PROTECTED REGION ID(inccpp1446033324019) ENABLED START*/ //Add additional includes here
ros::Publisher calibCoeff_pub;
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
        if (this->wm->rawSensorData.getOwnPositionVision() != NULL)
        {
            calibPosVisionX = this->wm->rawSensorData.getOwnPositionVision()->x;
            calibPosVisionY = this->wm->rawSensorData.getOwnPositionVision()->y;
        }

        auto calibRotation = this->wm->rawSensorData.getOwnVelocityMotion()->rotation;
        auto calibAngle = this->wm->rawSensorData.getOwnVelocityMotion()->angle;
        auto calibTranslation = this->wm->rawSensorData.getOwnVelocityMotion()->translation;

        calibPosMotionX = this->wm->rawSensorData.getOwnPositionMotion()->x;
        calibPosMotionY = this->wm->rawSensorData.getOwnPositionMotion()->y;
        auto calibPosMotionTheta = this->wm->rawSensorData.getOwnPositionMotion()->theta;

        std::cout << "======== CalcCalib ========" << std::endl;
        std::cout << "Motion: " << std::endl;
        std::cout << "Position: \tX: " << calibPosMotionX << "\tY: " << calibPosMotionY << "\tTheta: "
                << calibPosMotionTheta << std::endl;
        std::cout << "Velocity: \tRot: " << calibRotation << "\tAngle: " << calibAngle << "\tTrans: "
                << calibTranslation << std::endl;
//
//        correctedWayX = (calibPosMotionX - calibOldPosMotionX)
//                * cos(this->wm->rawSensorData.getOwnPositionVision(0)->theta
//                        - this->wm->rawSensorData.getOwnPositionMotion(0)->theta)
//                - (calibPosMotionY - calibOldPosMotionY)
//                        * sin(this->wm->rawSensorData.getOwnPositionVision(0)->theta
//                                - this->wm->rawSensorData.getOwnPositionMotion(0)->theta);
//        correctedWayY = (calibPosMotionX - calibOldPosMotionX)
//                * sin(this->wm->rawSensorData.getOwnPositionVision(0)->theta
//                        - this->wm->rawSensorData.getOwnPositionMotion(0)->theta)
//                + (calibPosMotionY - calibOldPosMotionY)
//                        * cos(this->wm->rawSensorData.getOwnPositionVision(0)->theta
//                                - this->wm->rawSensorData.getOwnPositionMotion(0)->theta);
//
//        correctedPosX = correctedPosX + correctedWayX;
//        correctedPosY = correctedPosY + correctedWayY;
//
//        lengthSegment = lengthSegment + sqrt((correctedWayX) * (correctedWayX) + (correctedWayY) * (correctedWayY));
//        length = length + sqrt((correctedWayX) * (correctedWayX) + (correctedWayY) * (correctedWayY));
//
//        calibOldPosMotionX = calibPosMotionX;
//        calibOldPosMotionY = calibPosMotionY;
//
//        if (tempyoyo == 5)
//        {
//            static int visionLengthCounter;
//            lengthVision = lengthVision
//                    + sqrt((calibOldPosVisionX - calibPosVisionX) * (calibOldPosVisionX - calibPosVisionX)
//                            + (calibOldPosVisionY - calibPosVisionY) * (calibOldPosVisionY - calibPosVisionY));
//
//            calibOldPosVisionX = calibPosVisionX;
//            calibOldPosVisionY = calibPosVisionY;
//            tempyoyo = 0;
//
//        }
//
//        tempyoyo++;
//
//        static int minusCounter = 0;
//        static int plusCounter = 0;
//
//        if (calibRotation > 0.1 || calibRotation < -0.1)
//        {
//
//            plusCounter++;
//        }
//        else
//        {
//
//            minusCounter++;
//        }

        // std::cout << "correctedWayX : " << correctedPosX << std::endl;
        // std::cout << "correctedWayY : " << correctedPosY << std::endl;
        // std::cout << "rotation : " << calibRotation << std::endl;
        // std::cout << "plusCounter : " << plusCounter << std::endl;
        // std::cout << "minusCounter : " << minusCounter << std::endl;
        // std::cout << "" << std::endl;

        //msl_actuator_msgs::MotionControl mc;
        //mc.motion.translation = 500;
        //send(mc);

        //}
        /*PROTECTED REGION END*/
    }
    void CalcCalib::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1446033324019) ENABLED START*/ //Add additional options here
        //initializePublisher();
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
            if (calibCounter == 3) //GonzalesUpdate
            {

                calibCoefficient *= calibSign(lengthVision, length)
                        * (sqrt(diffX * diffX + diffY * diffY) / lengthSegment) + 1; //GonzalesUpdate + lengthSegment

                if (calibCoefficient < 0.5)
                {
                    calibCoefficient = 1;
                }

            }

            string filename = string(sc->getConfigPath()) + string(sc->getHostname()) + string("/CalibData.txt");
            ofstream saveToCalibData;
            saveToCalibData.open(filename);
            saveToCalibData << calibCoefficient;
            saveToCalibData.close();

            ros::NodeHandle calibCEP;
            calibCoeff_pub = calibCEP.advertise < CalibrationCoefficient > ("CalibrationCoefficient", 1);

            calibCoeff.calibCoefficient = calibCoefficient;
            calibCoeff_pub.publish(calibCoeff);

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
                << calibSign(lengthVision, length) * (sqrt(diffX * diffX + diffY * diffY) / length) + 1 << std::endl;
        std::cout << "oldPosMotionX: " << calibOldPosMotionX << std::endl;
        std::cout << "posMotionX: " << calibPosMotionX << std::endl;
        std::cout << "lengthSegment: " << lengthSegment << std::endl;
        std::cout << "lengthVision: " << lengthVision << std::endl;
        std::cout << "ThetaVision: " << this->wm->rawSensorData.getOwnPositionVision(0)->theta << std::endl;
        std::cout << "ThetaMotion: " << this->wm->rawSensorData.getOwnPositionMotion(0)->theta << std::endl;

        std::cout << "" << std::endl;

        lengthSegment = 0;
        correctedPosX = this->wm->rawSensorData.getOwnPositionVision(0)->x;
        correctedPosY = this->wm->rawSensorData.getOwnPositionVision(0)->y;

        calibCounter++;

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1446033324019) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
