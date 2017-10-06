using namespace std;
#include "Plans/TwoHoledWall/AlignAndShootTwoHoledWall.h"

/*PROTECTED REGION ID(inccpp1417620683982) ENABLED START*/ //Add additional includes here
#include <math.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/kicker/Kicker.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1417620683982) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AlignAndShootTwoHoledWall::AlignAndShootTwoHoledWall() :
            DomainBehaviour("AlignAndShootTwoHoledWall")
    {
        /*PROTECTED REGION ID(con1417620683982) ENABLED START*/ //Add additional options here
//        this->setTrigger(&wm->visionTrigger);
//        maxVel = (*this->sc)["Show"]->get<double>("TwoHoledWall.MaxSpeed", NULL);
//
//        // Aiming/Rotation Stuff
//        lastRotError = 0;
//        timesOnTargetCounter = 0;
//        timesOnTargetThreshold = (*this->sc)["Show"]->get<int>("TwoHoledWall.TimesOnTarget", NULL);
//        wheelSpeed = (*this->sc)["Show"]->get<int>("TwoHoledWall.WheelSpeed", NULL);
//        pRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.RotationP", NULL);
//        dRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.RotationD", NULL);
//        minRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.MinRotation", NULL);
//        maxRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.MaxRotation", NULL);
//        angleTolerance = (*this->sc)["Show"]->get<double>("TwoHoledWall.AngleTolerance", NULL);
//        ballAngleTolerance = (*this->sc)["Show"]->get<double>("TwoHoledWall.BallAngleTolerance", NULL);
//
//        // Hole Stuff
//        lowerHole.x = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowerHole.X", NULL);
//        lowerHole.y = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowerHole.Y", NULL);
//        lowerHole.z = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowerHole.Z", NULL);
//
//        higherHole.x = (*this->sc)["Show"]->get<double>("TwoHoledWall.HigherHole.X", NULL);
//        higherHole.y = (*this->sc)["Show"]->get<double>("TwoHoledWall.HigherHole.Y", NULL);
//        higherHole.z = (*this->sc)["Show"]->get<double>("TwoHoledWall.HigherHole.Z", NULL);
//
//        holeMode = (HoleMode)(*this->sc)["Show"]->get<int>("TwoHoledWall.HoleMode", NULL);
//        useLowerHole = true;
//
//        // Kick Stuff
//        kicked = false;
//        iterationsAfterKick = 0;
//        voltage4shoot = (*this->sc)["Show"]->get<double>("TwoHoledWall.VoltageForShoot", NULL);
//        disableKicking = (*this->sc)["Show"]->get<bool>("TwoHoledWall.DisableKicking", NULL);
//
//        auto lowKickListSections = (*this->sc)["Show"]->getSections("TwoHoledWall", "LowKickList", NULL);
//        auto highKickListSections = (*this->sc)["Show"]->getSections("TwoHoledWall", "HighKickList", NULL);
//
//        // Load Kicking Power Lookup Lists for lower and higher hole
//        for (string sectionName : *lowKickListSections)
//        {
//            double distance = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowKickList", sectionName.c_str(),
//                                                               "distance", NULL);
//            double power = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowKickList", sectionName.c_str(), "power",
//                                                            NULL);
//            auto p = geometry::CNPointAllo (distance, power);
//            lowKickList.push_back(p);
//        }
//
//        for (string sectionName : *highKickListSections)
//        {
//            double distance = (*this->sc)["Show"]->get<double>("TwoHoledWall.HighKickList", sectionName.c_str(),
//                                                               "distance", NULL);
//            double power = (*this->sc)["Show"]->get<double>("TwoHoledWall.HighKickList", sectionName.c_str(), "power",
//                                                            NULL);
//            auto p = geometry::CNPointAllo (distance, power);
//            highKickList.push_back(p);
//        }
        /*PROTECTED REGION END*/
    }
    AlignAndShootTwoHoledWall::~AlignAndShootTwoHoledWall()
    {
        /*PROTECTED REGION ID(dcon1417620683982) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void AlignAndShootTwoHoledWall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1417620683982) ENABLED START*/ //Add additional options here
//        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent(); // actually ownPosition corrected
//        auto egoBallPos = wm->ball->getPositionEgo();
//
//        // stupid variant to be sure, that we have shoot!!!
//        if (kicked)
//        {
//            this->iterationsAfterKick++;
//            if (iterationsAfterKick > 30 && egoBallPos && egoBallPos->length() <= 400)
//            {
//                kicked = false;
//                iterationsAfterKick = 0;
//            }
//
//            if (!egoBallPos || egoBallPos->length() > 400)
//            {
//                if (holeMode == toggle)
//                {
//                    useLowerHole = !useLowerHole;
//                }
//                this->setSuccess(true);
//            }
//
//            return;
//        }
//
//        if (!ownPos|| !egoBallPos)
//        {
//            return;
//        }
//
//        // Constant ball handle wheel speed
//        msl_actuator_msgs::BallHandleCmd bhc;
//        bhc.leftMotor = (int8_t)this->wheelSpeed;
//        bhc.rightMotor = (int8_t)this->wheelSpeed;
//        send(bhc);
//
//        // Create ego-centric 2D target...
//        geometry::CNPointAllo alloHole(higherHole.x, higherHole.y);
//        if (useLowerHole)
//        {
//            alloHole.x = lowerHole.x;
//            alloHole.y = lowerHole.y;
//        }
//        geometry::CNPointEgo egoHole = alloHole.toEgo(*ownPos);
//
//        double egoHoleAngle = egoHole.angleZ();
//        double egoBallAngle = egoBallPos->angleZ();
//        double deltaHoleAngle = geometry::deltaAngle(egoHoleAngle, M_PI);
//        double deltaBallAngle = geometry::deltaAngle(egoBallAngle, M_PI);
//
//        // Counter for correct aiming
////		if (fabs(deltaHoleAngle) < this->angleTolerance)
//        if (fabs(deltaBallAngle) < this->ballAngleTolerance && fabs(deltaHoleAngle) < this->angleTolerance)
//        {
//            //cout << "align and shoot: hit target" << endl;
//            timesOnTargetCounter++;
//        }
//        else
//        {
//            //cout << "align and shoot: miss target" << endl;
//            timesOnTargetCounter = 0;
//        }
//
//        // Kick if aiming was correct long enough
//        if (timesOnTargetCounter > timesOnTargetThreshold)
//        {
//            msl_actuator_msgs::KickControl kc;
//            kc.enabled = true;
//            kc.kicker = egoBallPos->angleZ();
//            kc.power = setKickPower(egoHole.length());
//            float voltage;
//            if (!disableKicking)
//            {
//                send(kc);
//                kicked = true;
//                iterationsAfterKick = 0;
//                voltage = this->robot->kicker->getKickerVoltage();
//            }
//            else
//            {
//                // Send stop message to motion, in order to signal that the robot would shoot now
//                msl_actuator_msgs::MotionControl empty;
//                send(empty);
//                return;
//            }
//
//            std::cout << "AAShoot: Dist: " << egoHole.length() << "\tPower: " << kc.power << "\tDeviation: "
//                    << sin(deltaHoleAngle) * egoHole.length() << ",\tVolt: " << voltage << std::endl;
//            //this->success = true;
//            return;
//        }
//
//        // Create Motion Command for aiming
//        msl_actuator_msgs::MotionControl mc;
//
//        // PD Rotation Controller
//        mc.motion.rotation = -(deltaHoleAngle * pRot + (deltaHoleAngle - lastRotError) * dRot);
//        mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1)
//                * min(this->maxRot, max(fabs(mc.motion.rotation), this->minRot));
//
//        lastRotError = deltaHoleAngle;
//
//        // crate the motion orthogonal to the ball
//        geometry::CNPointEgo driveTo = egoBallPos->rotateZ(-M_PI / 2.0);
//        driveTo = driveTo * mc.motion.rotation;
//
//        // add the motion towards the ball
//        driveTo = driveTo + egoBallPos->normalize() * 10;
//
//        mc.motion.angle = driveTo.angleZ();
//        mc.motion.translation = min(this->maxVel, driveTo.length());
//
//        std::cout << "AAShoot: DeltaHoleAngle: " << deltaHoleAngle << "\tegoBall.X: " << egoBallPos->x << "\tegoBall.Y: "
//                << egoBallPos->y << "\tRotation: " << mc.motion.rotation << "\tDriveTo: (" << driveTo.x << ", "
//                << driveTo.y << ")" << std::endl;
//
//        send(mc);
        /*PROTECTED REGION END*/
    }
    void AlignAndShootTwoHoledWall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1417620683982) ENABLED START*/ //Add additional options here
//        timesOnTargetCounter = 0;
//        kicked = false;
//        iterationsAfterKick = 0;
//        switch (holeMode)
//        {
//            case toggle:
//                // We toggle in RUN-Methode before kicking.
//                break;
//            case lower:
//                useLowerHole = true;
//                break;
//            case upper:
//                useLowerHole = false;
//                break;
//        }
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1417620683982) ENABLED START*/ //Add additional methods here
    unsigned short AlignAndShootTwoHoledWall::setKickPower(double distance)
    {
//        vector < geometry::CNPointAllo> *kickList;
//        if (useLowerHole)
//        {
//            kickList = &lowKickList;
//        }
//        else
//        {
//            kickList = &highKickList;
//        }
//
//        int i = 0;
//        while (i < kickList->size() && distance > kickList->at(i).x)
//        {
//            i++;
//        }
//
//        // Don't interpolate for the first entry in the kick list ...
//        if (i == 0)
//        {
//            return kickList->at(0).y;
//        }
//
//        // Don't interpolate for the last entry in the kick list ...
//        if (i == kickList->size())
//        {
//            return kickList->at(kickList->size() - 1).y;
//        }
//
//        // Interpolate linear
//        return kickList->at(i - 1).y
//                + (distance - kickList->at(i - 1).x) / (kickList->at(i).x - kickList->at(i - 1).x)
//                        * (kickList->at(i).y - kickList->at(i - 1).y);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
