#ifndef DribbleCalibration_H_
#define DribbleCalibration_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1482339434271) ENABLED START*/ //Add additional includes here
#include "msl_robot/dribbleCalibration/container/DribbleCalibrationContainer.h"
#include "msl_robot/dribbleCalibration/container/MovementContainer.h"
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleCalibration : public DomainBehaviour
    {
    public:
        DribbleCalibration();
        virtual ~DribbleCalibration();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1482339434271) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1482339434271) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1482339434271) ENABLED START*/ //Add additional private methods here
        /**
         * if true -> robot will dribble forward in the calibration process
         */
        bool runForwardCal;
        /**
         * if true -> robot will dribble backward in the calibration process
         */
        bool runBackwardCal;

        /**
         * variable for dribble forward parameter detection
         */
        bool dribbleForward;
        /**
         * variable for dribble backward parameter detection
         */
        bool dribbleBackward;
        /**
         * variable for dribble rotation parameter detection
         */
        bool dribbleRotateLeft;
        /**
         * variable for dribble rotation parameter detection
         */
        bool dribbleRotateRight;

        msl::DribbleCalibrationContainer dcc;
        msl::DribbleCalibrationContainer::Param param;
        msl::MovementContainer moveCont;

        // for speed iterations
        double startTrans;
        double endTrans;
        int speedIter;
        int moveCount;
        int getBallCount;

        // output flags
        bool getBallFlag;
        bool motionCmdFlag;
        bool transFlag;

        // for movement
        int haveBallCount;
        int haveBallWaitingDuration;
        int collectDataWaitingDuration;

        // param adaption stuff
        int minHaveBallIter;

        void readConfigParameters();
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleCalibration_H_ */
