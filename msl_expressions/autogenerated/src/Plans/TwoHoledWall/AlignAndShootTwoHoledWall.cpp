using namespace std;
#include "Plans/TwoHoledWall/AlignAndShootTwoHoledWall.h"

/*PROTECTED REGION ID(inccpp1417620683982) ENABLED START*/ //Add additional includes here
#include <math.h>
#include <RawSensorData.h>
#include <Ball.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1417620683982) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AlignAndShootTwoHoledWall::AlignAndShootTwoHoledWall() :
            DomainBehaviour("AlignAndShootTwoHoledWall")
    {
        /*PROTECTED REGION ID(con1417620683982) ENABLED START*/ //Add additional options here
        this->setTrigger(&wm->visionTrigger);
        maxVel = (*this->sc)["Show"]->get<double>("TwoHoledWall.MaxSpeed", NULL);

        // Aiming/Rotation Stuff
        lastRotError = 0;
        timesOnTargetCounter = 0;
        timesOnTargetThreshold = (*this->sc)["Show"]->get<int>("TwoHoledWall.TimesOnTarget", NULL);
        wheelSpeed = (*this->sc)["Show"]->get<int>("TwoHoledWall.WheelSpeed", NULL);
        pRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.RotationP", NULL);
        dRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.RotationD", NULL);
        minRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.MinRotation", NULL);
        maxRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.MaxRotation", NULL);
        angleTolerance = (*this->sc)["Show"]->get<double>("TwoHoledWall.AngleTolerance", NULL);
        ballAngleTolerance = (*this->sc)["Show"]->get<double>("TwoHoledWall.BallAngleTolerance", NULL);

        // Hole Stuff
        lowerHole.x = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowerHole.X", NULL);
        lowerHole.y = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowerHole.Y", NULL);
        lowerHole.z = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowerHole.Z", NULL);

        higherHole.x = (*this->sc)["Show"]->get<double>("TwoHoledWall.HigherHole.X", NULL);
        higherHole.y = (*this->sc)["Show"]->get<double>("TwoHoledWall.HigherHole.Y", NULL);
        higherHole.z = (*this->sc)["Show"]->get<double>("TwoHoledWall.HigherHole.Z", NULL);

        holeMode = (HoleMode)(*this->sc)["Show"]->get<int>("TwoHoledWall.HoleMode", NULL);
        useLowerHole = true;

        // Kick Stuff
        voltage4shoot = (*this->sc)["Show"]->get<double>("TwoHoledWall.VoltageForShoot", NULL);
        disableKicking = (*this->sc)["Show"]->get<bool>("TwoHoledWall.DisableKicking", NULL);

        auto lowKickListSections = (*this->sc)["Show"]->getSections("TwoHoledWall", "LowKickList", NULL);
        auto highKickListSections = (*this->sc)["Show"]->getSections("TwoHoledWall", "HighKickList", NULL);

        // Load Kicking Power Lookup Lists for lower and higher hole
        for (string sectionName : *lowKickListSections)
        {
            double distance = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowKickList", sectionName.c_str(),
                                                               "distance", NULL);
            double power = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowKickList", sectionName.c_str(), "power",
                                                            NULL);
            auto p = make_shared < geometry::CNPoint2D > (distance, power);
            lowKickList.push_back(p);
        }

        for (string sectionName : *highKickListSections)
        {
            double distance = (*this->sc)["Show"]->get<double>("TwoHoledWall.HighKickList", sectionName.c_str(),
                                                               "distance", NULL);
            double power = (*this->sc)["Show"]->get<double>("TwoHoledWall.HighKickList", sectionName.c_str(), "power",
                                                            NULL);
            auto p = make_shared < geometry::CNPoint2D > (distance, power);
            highKickList.push_back(p);
        }
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
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision(); // actually ownPosition corrected
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();

        // stupid variant to be sure, that we have shoot!!!
        if (kicked)
        {
            this->iterationsAfterKick++;
            if (iterationsAfterKick > 30 && egoBallPos != nullptr && egoBallPos->length() <= 400)
            {
                kicked = false;
                iterationsAfterKick = 0;
            }

            if (egoBallPos == nullptr || egoBallPos->length() > 400)
            {
                if (holeMode == toggle)
                {
                    useLowerHole = !useLowerHole;
                }
                this->setSuccess(true);
            }

            return;
        }

        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        // Constant ball handle wheel speed
        BallHandleCmd bhc;
        bhc.leftMotor = (int8_t)this->wheelSpeed;
        bhc.rightMotor = (int8_t)this->wheelSpeed;
        send(bhc);

        // Create ego-centric 2D target...
        shared_ptr < geometry::CNPoint2D > egoHole;
        geometry::CNPoint2D alloHole(higherHole.x, higherHole.y);
        if (useLowerHole)
        {
            alloHole.x = lowerHole.x;
            alloHole.y = lowerHole.y;
        }
        egoHole = alloHole.alloToEgo(*ownPos);

        double egoHoleAngle = egoHole->angleTo();
        double egoBallAngle = egoBallPos->angleTo();
        double deltaHoleAngle = geometry::deltaAngle(egoHoleAngle, M_PI);
        double deltaBallAngle = geometry::deltaAngle(egoBallAngle, M_PI);

        // Counter for correct aiming
//		if (fabs(deltaHoleAngle) < this->angleTolerance)
        if (fabs(deltaBallAngle) < this->ballAngleTolerance && fabs(deltaHoleAngle) < this->angleTolerance)
        {
            //cout << "align and shoot: hit target" << endl;
            timesOnTargetCounter++;
        }
        else
        {
            //cout << "align and shoot: miss target" << endl;
            timesOnTargetCounter = 0;
        }

        // Kick if aiming was correct long enough
        if (timesOnTargetCounter > timesOnTargetThreshold)
        {
            KickControl kc;
            kc.enabled = true;
            kc.kicker = egoBallPos->angleTo();
            kc.power = setKickPower(egoHole->length());
            float voltage;
            if (!disableKicking)
            {
                send(kc);
                kicked = true;
                iterationsAfterKick = 0;
                voltage = wm->getKickerVoltage();
            }
            else
            {
                // Send stop message to motion, in order to signal that the robot would shoot now
                MotionControl empty;
                send(empty);
                return;
            }

            cout << "AAShoot: Dist: " << egoHole->length() << "\tPower: " << kc.power << "\tDeviation: "
                    << sin(deltaHoleAngle) * egoHole->length() << ",\tVolt: " << voltage << endl;
            //this->success = true;
            return;
        }

        // Create Motion Command for aiming
        MotionControl mc;

        // PD Rotation Controller
        mc.motion.rotation = -(deltaHoleAngle * pRot + (deltaHoleAngle - lastRotError) * dRot);
        mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1)
                * min(this->maxRot, max(fabs(mc.motion.rotation), this->minRot));

        lastRotError = deltaHoleAngle;

        // crate the motion orthogonal to the ball
        shared_ptr < geometry::CNPoint2D > driveTo = egoBallPos->rotate(-M_PI / 2.0);
        driveTo = driveTo * mc.motion.rotation;

        // add the motion towards the ball
        driveTo = driveTo + egoBallPos->normalize() * 10;

        mc.motion.angle = driveTo->angleTo();
        mc.motion.translation = min(this->maxVel, driveTo->length());

        cout << "AAShoot: DeltaHoleAngle: " << deltaHoleAngle << "\tegoBall.X: " << egoBallPos->x << "\tegoBall.Y: "
                << egoBallPos->y << "\tRotation: " << mc.motion.rotation << "\tDriveTo: (" << driveTo->x << ", "
                << driveTo->y << ")" << endl;

        send(mc);
        /*PROTECTED REGION END*/
    }
    void AlignAndShootTwoHoledWall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1417620683982) ENABLED START*/ //Add additional options here
        timesOnTargetCounter = 0;
        kicked = false;
        iterationsAfterKick = 0;
        switch (holeMode)
        {
            case toggle:
                // We toggle in RUN-Methode before kicking.
                break;
            case lower:
                useLowerHole = true;
                break;
            case upper:
                useLowerHole = false;
                break;
        }
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1417620683982) ENABLED START*/ //Add additional methods here
    unsigned short AlignAndShootTwoHoledWall::setKickPower(double distance)
    {
        vector < shared_ptr < geometry::CNPoint2D >> *kickList;
        if (useLowerHole)
        {
            kickList = &lowKickList;
        }
        else
        {
            kickList = &highKickList;
        }

        int i = 0;
        while (i < kickList->size() && distance > kickList->at(i)->x)
        {
            i++;
        }

        // Don't interpolate for the first entry in the kick list ...
        if (i == 0)
        {
            return kickList->at(0)->y;
        }

        // Don't interpolate for the last entry in the kick list ...
        if (i == kickList->size())
        {
            return kickList->at(kickList->size() - 1)->y;
        }

        // Interpolate linear
        return kickList->at(i - 1)->y
                + (distance - kickList->at(i - 1)->x) / (kickList->at(i)->x - kickList->at(i - 1)->x)
                        * (kickList->at(i)->y - kickList->at(i - 1)->y);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
