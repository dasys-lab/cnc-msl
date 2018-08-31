using namespace std;
#include "Plans/TwoHoledWall/AlignAndShootTwoHoledWall.h"

/*PROTECTED REGION ID(inccpp1417620683982) ENABLED START*/ //Add additional includes here
#include <math.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/kicker/Kicker.h>

#include <LaserScanner.h>
#include <msl_sensor_msgs/LaserLocalization.h>
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
        this->maxVel = (*this->sc)["Show"]->get<double>("TwoHoledWall.MaxSpeed", NULL);

        // Aiming/Rotation Stuff
        this->lastRotError = 0;
        this->timesOnTargetCounter = 0;
        this->timesOnTargetThreshold = (*this->sc)["Show"]->get<int>("TwoHoledWall.TimesOnTarget", NULL);
        this->wheelSpeed = (*this->sc)["Show"]->get<int>("TwoHoledWall.WheelSpeed", NULL);
        this->pRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.RotationP", NULL);
        this->dRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.RotationD", NULL);
        this->minRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.MinRotation", NULL);
        this->maxRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.MaxRotation", NULL);
        this->angleTolerance = (*this->sc)["Show"]->get<double>("TwoHoledWall.AngleTolerance", NULL);
        this->ballAngleTolerance = (*this->sc)["Show"]->get<double>("TwoHoledWall.BallAngleTolerance", NULL);


        // Localize goal with laser scanner
//        auto ownPosition = wm->rawSensorData->getOwnPositionVision();
//		if (ownPosition == nullptr)
//		{
//			return;
//		}
//		auto position = wm->laserScanner->getGoalWallPosition();
//		if (position == nullptr)
//		{
//			return;
//		}
//		geometry::CNPoint2D egoLeftGoalPos = geometry::CNPoint2D(position->points[0].x, position->points[0].y);
//		geometry::CNPoint2D egoRightGoalPos = geometry::CNPoint2D(position->points[1].x, position->points[1].y);
//
//		auto alloLeftGoalPos = egoLeftGoalPos.egoToAllo(*ownPosition);
//		auto alloRighGoalPos = egoRightGoalPos.egoToAllo(*ownPosition);
//
//		// Goal position based on configuration
//		auto configAlloLeftGoalPos = wm->field->posLeftOppGoalPost();
//		auto configAlloRightGoalPos = wm->field->posRightOppGoalPost();

        // Hole Stuff
        this->lowerHole.x = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowerHole.X", NULL);
        this->lowerHole.y = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowerHole.Y", NULL);
        this->lowerHole.z = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowerHole.Z", NULL);

        this->higherHole.x = (*this->sc)["Show"]->get<double>("TwoHoledWall.HigherHole.X", NULL);
        this->higherHole.y = (*this->sc)["Show"]->get<double>("TwoHoledWall.HigherHole.Y", NULL);
        this->higherHole.z = (*this->sc)["Show"]->get<double>("TwoHoledWall.HigherHole.Z", NULL);

        // Modify hole position via laser scan localization
//        cout << "f[" << this->higherHole.x << ',' << this->higherHole.y << std::endl;
//        cout << "c[" << configAlloLeftGoalPos->x << ',' << configAlloLeftGoalPos->y << std::endl;
//        cout << "o[" << alloLeftGoalPos->x << ',' << alloRighGoalPos->y << std::endl;
        //this->lowerHole.x = configAlloRightGoalPos - (configAlloRightGoalPos.x - this->lowerHole.x);
        //this->lowerHole.y = configAlloRightGoalPos - (configAlloRightGoalPos.y - this->lowerHole.y);

        this->holeMode = (HoleMode)(*this->sc)["Show"]->get<int>("TwoHoledWall.HoleMode", NULL);
        this->useLowerHole = true;

        // Kick Stuff
        this->voltage4shoot = (*this->sc)["Show"]->get<double>("TwoHoledWall.VoltageForShoot", NULL);
        this->disableKicking = (*this->sc)["Show"]->get<bool>("TwoHoledWall.DisableKicking", NULL);

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
            this->lowKickList.push_back(p);
        }

        for (string sectionName : *highKickListSections)
        {
            double distance = (*this->sc)["Show"]->get<double>("TwoHoledWall.HighKickList", sectionName.c_str(),
                                                               "distance", NULL);
            double power = (*this->sc)["Show"]->get<double>("TwoHoledWall.HighKickList", sectionName.c_str(), "power",
                                                            NULL);
            auto p = make_shared < geometry::CNPoint2D > (distance, power);
            this->highKickList.push_back(p);
        }
        this->iterationsAfterKick = 0;
        this->kicked = false;
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
        shared_ptr < geometry::CNPosition > ownPos = this->wm->rawSensorData->getOwnPositionVision(); // actually ownPosition corrected
        shared_ptr < geometry::CNPoint2D > egoBallPos = this->wm->ball->getEgoBallPosition();

        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        // Localize goal with laser scanner

		auto position = wm->laserScanner->getGoalWallPosition();
		if (position == nullptr)
		{
			return;
		}
		geometry::CNPoint2D egoLeftGoalPos = geometry::CNPoint2D(position->points[0].x, position->points[0].y);
		geometry::CNPoint2D egoRightGoalPos = geometry::CNPoint2D(position->points[1].x, position->points[1].y);

		auto alloLeftGoalPos = egoLeftGoalPos.egoToAllo(*ownPos);
		auto alloRighGoalPos = egoRightGoalPos.egoToAllo(*ownPos);

		// Goal position based on configuration
		auto configAlloLeftGoalPos = wm->field->posLeftOppGoalPost();
		auto configAlloRightGoalPos = wm->field->posRightOppGoalPost();

		// Modify hole position via laser scan localization
		cout << "f[" << this->lowerHole.x << ',' << this->lowerHole.y << std::endl;
		cout << "c[" << configAlloRightGoalPos->x << ',' << configAlloRightGoalPos->y << std::endl;
		cout << "o[" << alloRighGoalPos->x << ',' << alloRighGoalPos->y << std::endl;

        // stupid variant to be sure, that we have shoot!!!
        if (this->kicked)
        {
            this->iterationsAfterKick++;
            if (this->iterationsAfterKick > 30 && egoBallPos != nullptr && egoBallPos->length() <= 400)
            {
                this->kicked = false;
                this->iterationsAfterKick = 0;
            }

            if (egoBallPos == nullptr || egoBallPos->length() > 400)
            {
                if (this->holeMode == toggle)
                {
                    this->useLowerHole = !this->useLowerHole;
                }
                this->setSuccess(true);
            }
            cout << "AASTHW: return 1" << endl;
            return;
        }

        //replaced with DribbleConstTHW
//        // Constant ball handle wheel speed
//        BallHandleCmd bhc;
////        bhc.leftMotor = (int8_t)this->wheelSpeed;
////        bhc.rightMotor = (int8_t)this->wheelSpeed;
//        bhc.leftMotor = this->wheelSpeed;
//        bhc.rightMotor = this->wheelSpeed;
//        send(bhc);

        // Create ego-centric 2D target...
        shared_ptr < geometry::CNPoint2D > egoHole;
        geometry::CNPoint2D alloHole(this->higherHole.x, this->higherHole.y);
        if (useLowerHole)
        {
            alloHole.x = this->lowerHole.x;
            alloHole.y = this->lowerHole.y;
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
            cout << "AlignAndShootTwoHoledWall: hit target" << endl;
            this->timesOnTargetCounter++;
        }
        else
        {
//            cout << "AlignAndShootTwoHoledWall: miss target: fabs(deltaBallAngle): " << fabs(deltaBallAngle)
//                    << "ballAngleTolerance: " << this->ballAngleTolerance << "fabs(deltaHoleAngle): "
//                    << fabs(deltaHoleAngle) << "angleToleranc: " << this->angleTolerance << endl;
            this->timesOnTargetCounter = 0;
        }

        cout << "timesOnTargetCounter: " << this->timesOnTargetCounter << endl;
        cout << "timesOnTargetThreshold: " << this->timesOnTargetThreshold << endl;
        // Kick if aiming was correct long enough
        if (this->timesOnTargetCounter > this->timesOnTargetThreshold)
        {
            KickControl kc;
            kc.enabled = true;
//            kc.kicker = egoBallPos->angleTo();
            cout << "AlignAndShootTwoHoledWall: dist to hole: " << egoHole->length() << endl;
            kc.power = setKickPower(egoHole->length());
            float voltage;
            if (!this->disableKicking)
            {
                send(kc);
                this->kicked = true;
                this->iterationsAfterKick = 0;
                voltage = this->robot->kicker->getKickerVoltage();
                cout << "voltage: " << voltage << endl;
            }
            else
            {
                // Send stop message to motion, in order to signal that the robot would shoot now
                MotionControl empty;
                empty.motion.angle = 0;
                empty.motion.rotation = 0;
                empty.motion.translation = 0;
                send(empty);
                cout << "return disablelkicking" << endl;
                return;
            }

//            cout << "AAShoot: Dist: " << egoHole->length() << "\tPower: " << kc.power << "\tDeviation: "
//                    << sin(deltaHoleAngle) * egoHole->length() << ",\tVolt: " << voltage << endl;
            this->setSuccess(true);
//            cout << "return after kick" << endl;
            return;
            //return;
        }

        // Create Motion Command for aiming
        MotionControl mc;

        // PD Rotation Controller
        mc.motion.rotation = -(deltaHoleAngle * this->pRot + (deltaHoleAngle - lastRotError) * this->dRot);
        mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1)
                * min(this->maxRot, max(fabs(mc.motion.rotation), this->minRot));

        this->lastRotError = deltaHoleAngle;

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
        this->timesOnTargetCounter = 0;
        this->kicked = false;
        this->iterationsAfterKick = 0;
        this->lastRotError = 0;
        switch (this->holeMode)
        {
            case toggle:
                // We toggle in RUN-Methode before kicking.
                break;
            case lower:
                this->useLowerHole = true;
                break;
            case upper:
                this->useLowerHole = false;
                break;
        }

        // Localize goal with laser scanner
        auto ownPosition = wm->rawSensorData->getOwnPositionVision();
		if (ownPosition == nullptr)
		{
			return;
		}

		auto position = wm->laserScanner->getGoalWallPosition();
		if (position == nullptr)
		{
			return;
		}
		geometry::CNPoint2D egoLeftGoalPos = geometry::CNPoint2D(position->points[0].x, position->points[0].y);
		geometry::CNPoint2D egoRightGoalPos = geometry::CNPoint2D(position->points[1].x, position->points[1].y);

		auto alloLeftGoalPos = egoLeftGoalPos.egoToAllo(*ownPosition);
		auto alloRighGoalPos = egoRightGoalPos.egoToAllo(*ownPosition);

		// Goal position based on configuration
		auto configAlloLeftGoalPos = wm->field->posLeftOppGoalPost();
		auto configAlloRightGoalPos = wm->field->posRightOppGoalPost();

		// Modify hole position via laser scan localization
		cout << "f[" << this->lowerHole.x << ',' << this->lowerHole.y << std::endl;
		cout << "c[" << configAlloRightGoalPos->x << ',' << configAlloRightGoalPos->y << std::endl;
		cout << "o[" << alloRighGoalPos->x << ',' << alloRighGoalPos->y << std::endl;

		//this->lowerHole.x = configAlloRightGoalPos - (configAlloRightGoalPos.x - this->lowerHole.x);
		//this->lowerHole.y = configAlloRightGoalPos - (configAlloRightGoalPos.y - this->lowerHole.y);
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1417620683982) ENABLED START*/ //Add additional methods here
    unsigned short AlignAndShootTwoHoledWall::setKickPower(double distance)
    {
        vector < shared_ptr < geometry::CNPoint2D >> *kickList;
        if (this->useLowerHole)
        {
            kickList = &this->lowKickList;
        }
        else
        {
            kickList = &this->highKickList;
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
