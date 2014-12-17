using namespace std;
#include "Plans/TwoHoledWall/AlignAndShootTwoHoledBall.h"

/*PROTECTED REGION ID(inccpp1417620683982) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1417620683982) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AlignAndShootTwoHoledBall::AlignAndShootTwoHoledBall() :
            DomainBehaviour("AlignAndShootTwoHoledBall")
    {
        /*PROTECTED REGION ID(con1417620683982) ENABLED START*/ //Add additional options here
    	field = MSLFootballField::getInstance();
    	timesOnTarget=0;

		maxVel= 2000;
		pRot = 2.1;
		dRot = 0.0;
		lastRotError = 0;
		minRot = 0.1;
		maxRot = M_PI*4;
		angleTolerance = 0.05;
		disableKicking = false;

		usedFixedHole = false;
		useLowerHoleFixed = false;
		shootingSpeed = 300.0;
		TIMES_ON_TARGET = 1;
		wheelSpeed = -40;
		voltage4shoot = 328.0;
    	double x,y,z;
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

		if( usedFixedHole )
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

		/*List<Point2D> lowKick = new List<Point2D>();
		try
		{
			int i=1;
			while( true )
			{
				double distance = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowKickList.p" + i + ".distance", NULL);
				double power = (*this->sc)["Show"]->get<double>("TwoHoledWall.LowKickList.p" + i + ".power", NULL);
				lowKick.Add(new Point2D(distance,power));
				i++;
			}
		}
		catch(Exception)
		{
			this.lowKickList = lowKick.ToArray();
		}

    				List<Point2D> highKick = new List<Point2D>();
    				try
    				{
    					int i=1;
    					while( true )
    					{
    						double distance = (*this->sc)["Show"]->get<double>("TwoHoledWall.HighKickList.p" + i + ".distance", NULL);
    						double power = (*this->sc)["Show"]->get<double>("TwoHoledWall.HighKickList.p" + i + ".power", NULL);
    						highKick.Add(new Point2D(distance,power));
    						i++;
    					}
    				}
    				catch(Exception)
    				{
    					this.highKickList = highKick.ToArray();
    				}
    				*/
        /*PROTECTED REGION END*/
    }
    AlignAndShootTwoHoledBall::~AlignAndShootTwoHoledBall()
    {
        /*PROTECTED REGION ID(dcon1417620683982) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void AlignAndShootTwoHoledBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1417620683982) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void AlignAndShootTwoHoledBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1417620683982) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1417620683982) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
