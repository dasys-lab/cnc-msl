using namespace std;
#include "Plans/TwoHoledWall/AlignAndShootTwoHoledWall.h"

/*PROTECTED REGION ID(inccpp1417620683982) ENABLED START*/ //Add additional includes here
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
        field = MSLFootballField::getInstance();
        timesOnTarget = 0;
        changeHole = false;

        maxVel = 2000;
        pRot = 2.1;
        dRot = 0.0;
        lastRotError = 0;
        minRot = 0.1;
        maxRot = M_PI * 4;
        angleTolerance = 0.05;
        disableKicking = false;

        usedFixedHole = false;
        useLowerHoleFixed = false;
        shootingSpeed = 300.0;
        TIMES_ON_TARGET = 1;
        wheelSpeed = -40;
        voltage4shoot = 328.0;
        double x, y, z;
        (*this->sc)["Drive"]->get<double>("Drive", "DefaultVelocity", NULL);

        lowerHole.x = (*this->sc)["Show"]->get<double>("TwoHoledWall", "LowerHole", "X", NULL);
        lowerHole.y = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowerHole.Y", NULL);
        lowerHole.z = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowerHole.Z", NULL);

        higherHole.x = (*this->sc)["Show"]->get<double>("TwoHoledWall.HigherHole.X", NULL);
        higherHole.y = (*this->sc)["Show"]->get<double>("TwoHoledWall.HigherHole.Y", NULL);
        higherHole.z = (*this->sc)["Show"]->get<double>("TwoHoledWall.HigherHole.Z", NULL);

        usedFixedHole = (*this->sc)["Show"]->get<bool>("TwoHoledWall.UseFixedHole", NULL);
        useLowerHoleFixed = (*this->sc)["Show"]->get<bool>("TwoHoledWall.UseLowerHoleFixed", NULL);
        shootingSpeed = (*this->sc)["Show"]->get<double>("TwoHoledWall.ShootingSpeed", NULL);

        if (usedFixedHole)
        {
            useLowerHole = useLowerHoleFixed;
        }

        TIMES_ON_TARGET = (*this->sc)["Show"]->get<int>("TwoHoledWall.TimesOnTarget", NULL);
        wheelSpeed = (*this->sc)["Show"]->get<int>("TwoHoledWall.WheelSpeed", NULL);
        voltage4shoot = (*this->sc)["Show"]->get<double>("TwoHoledWall.VoltageForShoot", NULL);

        maxVel = (*this->sc)["Show"]->get<double>("TwoHoledWall.MaxSpeed", NULL);
        pRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.RotationP", NULL);
        dRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.RotationD", NULL);
        minRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.MinRotation", NULL);
        maxRot = (*this->sc)["Show"]->get<double>("TwoHoledWall.MaxRotation", NULL);
        angleTolerance = (*this->sc)["Show"]->get<double>("TwoHoledWall.AngleTolerance", NULL);
        disableKicking = (*this->sc)["Show"]->get<bool>("TwoHoledWall.DisableKicking", NULL);

        auto lowKickSectionList((*this->sc)["Show"]->getSections("TwoHoledWall", "LowKickList", NULL));

        lowKickList.reserve(lowKickSectionList->size());

        auto highKickSectionList((*this->sc)["Show"]->getSections("TwoHoledWall", "HighKickList", NULL));

        highKickList.reserve(highKickSectionList->size());
        try
        {
            int i = 1;
            while (true)
            {
                bool found = false;
                string currentSearchString = string("p") + std::to_string(i);
                for (int j = 0; j < lowKickSectionList->size(); j++)
                {
                    if (lowKickSectionList->at(j) == currentSearchString)
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    break;
                }

                stringstream s1;
                s1 << "TwoHoledWall.LowKickList.p" << i << ".distance";

                stringstream s2;
                s2 << "TwoHoledWall.LowKickList.p" << i << ".power";

                double distance = (*this->sc)["Show"]->get<double>(s1.str().c_str(), NULL);
                double power = (*this->sc)["Show"]->get<double>(s2.str().c_str(), NULL);
                auto p = make_shared < CNPoint2D > (distance, power);
                cout << "align and shoot: " << p->x << " " << p->y << endl;
                lowKickList.push_back(p);
                i++;
            }

            i = 1;
            while (true)
            {
                bool found = false;
                string currentSearchString = string("p") + std::to_string(i);
                for (int j = 0; j < highKickSectionList->size(); j++)
                {
                    if (highKickSectionList->at(j) == currentSearchString)
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    break;
                }

                stringstream s1;
                s1 << "TwoHoledWall.HighKickList.p" << i << ".distance";

                stringstream s2;
                s2 << "TwoHoledWall.HighKickList.p" << i << ".power";

                double distance = (*this->sc)["Show"]->get<double>(s1.str().c_str(), NULL);
                double power = (*this->sc)["Show"]->get<double>(s2.str().c_str(), NULL);
                auto p = make_shared < CNPoint2D > (distance, power);
                highKickList.push_back(p);
                i++;
            }
        }
        catch (exception e)
        {
            cerr << "Error loading parameters in Behaviour AlignAndShootTwoHoledWall";
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
        double volt = wm->getKickerVoltage();
        shared_ptr < CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision(); // actually ownPosition corrected
        shared_ptr < CNPoint2D > ballPos = wm->ball.getEgoBallPosition();
        if (ownPos == nullptr || ballPos == nullptr)
        {
            return;
        }
//		CNVelocity2D ballVel = WM.BallVelocity;
//		CNPoint2D ballVel2;
//
//		if (ballVel == null) {
//			ballVel2 = new CNPoint2D(0, 0);
//		} else if (ballVel.Length() > 5000) {
//			Velocity v = ballVel.Normalize()*5000;
//			ballVel2 = new CNPoint2D(v.Vx,v.Vy);
//		}
//		else {
//			ballVel2 = new CNPoint2D(ballVel.Vx,ballVel.Vy);
//		}

        shared_ptr < CNPoint2D > egoTarget;

        if (useLowerHole)
        {
            CNPoint2D alloTarget(lowerHole.x, lowerHole.y);
            egoTarget = alloTarget.alloToEgo(*ownPos);
        }
        else
        {
            CNPoint2D alloTarget(higherHole.x, higherHole.y);
            egoTarget = alloTarget.alloToEgo(*ownPos);
        }

        //cout << "egoTarget : " << egoTarget->x << " " << egoTarget->y << endl;

        double aimAngle = egoTarget->angleTo();

        double ballAngle = ballPos->angleTo();

        double deltaAngle = GeometryCalculator::deltaAngle(ballAngle, aimAngle);

        MotionControl mc;
        mc.motion.rotation = deltaAngle * pRot + (deltaAngle - lastRotError) * dRot;

        BallHandleCmd bhc;
        bhc.leftMotor = (int8_t)this->wheelSpeed;
        bhc.rightMotor = (int8_t)this->wheelSpeed;

        //cout << "DeltaAngle is : " << deltaAngle << " ball angle : " << ballAngle << " aimAngle : " << aimAngle << endl;

        send(bhc);

        if (fabs(deltaAngle) < this->angleTolerance)
        {
            //cout << "align and shoot: hit target" << endl;
            timesOnTarget++;
        }
        else
        {
            //cout << "align and shoot: miss target" << endl;
            timesOnTarget = 0;
        }

        float voltage = wm->getKickerVoltage();
        //	cout << "align and shoot: " << " " << timesOnTarget <<" " <<  TIMES_ON_TARGET << endl;
        if (timesOnTarget > TIMES_ON_TARGET/* && fabs(this->voltage4shoot-voltage) < 1.001*/)
        {
            //KICK!
            KickControl kc;
            kc.enabled = true;
            kc.kicker = ballPos->angleTo();
            kc.power = (ushort)(setKickPower(egoTarget->length(), (useLowerHole ? lowerHole.z : higherHole.z)));
            if (!disableKicking)
            {
                cout << "align and shoot: sending kc!" << endl;
                changeHole = true;
                send(kc);
            }
            else
            {
                MotionControl empty;
                send(empty);
                return;
            }
            mc.motion.rotation = 0;
            this->success = true;
            cout << "Dist: " << egoTarget->length() << "\tPower: " << kc.power << "\tDeviation: "
                    << sin(deltaAngle) * egoTarget->length() << ",\tVolt: " << volt << endl;

        }
        else
        {
            mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1)
                    * min(this->maxRot, max(fabs(mc.motion.rotation), this->minRot));
        }
        lastRotError = deltaAngle;

        double transBallOrth = ballPos->length() * mc.motion.rotation; //may be negative!
        double transBallTo = ballPos->length(); //max(ballPos->length(),ballVel2.Distance());

        shared_ptr < CNPoint2D > driveTo = ballPos->rotate((-M_PI / 2.0));
        driveTo = driveTo->normalize() * transBallOrth;
        driveTo = driveTo + ballPos->normalize() * transBallTo;

        if (driveTo->length() > maxVel)
        {
            driveTo = driveTo->normalize() * maxVel;
        }

        mc.motion.angle = driveTo->angleTo();
        mc.motion.translation = this->shootingSpeed; //driveTo.Distance();
        cout << "Align and shoot: " << "delta angle " << deltaAngle << " mc.motion.rotation " << mc.motion.rotation
                << " driveto " << driveTo->x << " " << driveTo->y << endl;
        send(mc);
        /*PROTECTED REGION END*/
    }
    void AlignAndShootTwoHoledWall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1417620683982) ENABLED START*/ //Add additional options here
        timesOnTarget = 0;
        if (!usedFixedHole && changeHole)
        {
            useLowerHole = !useLowerHole;
        }
        changeHole = false;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1417620683982) ENABLED START*/ //Add additional methods here
    double AlignAndShootTwoHoledWall::interPolatePower(double dist, vector<shared_ptr<CNPoint2D> > values)
    {
        int i = 0;
        while (i < values.size() && dist > values[i]->x)
        {
            i++;
        }
        if (i == 0)
        {
            return values[0]->y;
        }
        if (i == values.size())
        {
            return values[values.size() - 1]->y;
        }
        return values[i - 1]->y
                + (dist - values[i - 1]->x) / (values[i]->x - values[i - 1]->x) * (values[i]->y - values[i - 1]->y);
    }

    unsigned short AlignAndShootTwoHoledWall::setKickPower(double distance, double height)
    {
        if (height > 600)
        {
            return (unsigned short)interPolatePower(distance, highKickList);
        }
        else
        {
            return (unsigned short)interPolatePower(distance, lowKickList);
        }
    }
/*PROTECTED REGION END*/
} /* namespace alica */
