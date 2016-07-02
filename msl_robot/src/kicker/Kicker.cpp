/*
 * Kicker.cpp
 *
 *  Created on: Jul 13, 2015
 *      Author: Stefan Jakob
 */

#include <msl_robot/kicker/Kicker.h>
#include <MSLWorldModel.h>
#include <Game.h>
#include <msl_sensor_msgs/ObstacleInfo.h>
#include <obstaclehandler/Obstacles.h>
#include <GeometryCalculator.h>
#include <Ball.h>
#include <RawSensorData.h>

namespace msl
{

	double Kicker::kickerAngle = M_PI;

	Kicker::Kicker(MSLWorldModel* wm) : kickControlMsgs(10)
	{
		this->wm = wm;
		this->sc = supplementary::SystemConfig::getInstance();
		this->TWO_THIRD_PI = 2 * M_PI / 3;
		this->kickerCount = (*this->sc)["KickHelper"]->get<double>("KickConfiguration", "KickerCount", NULL);
		this->ballisticCurveOffset = (*this->sc)["KickHelper"]->get<double>("KickConfiguration", "BallisticCurveOffset",
		NULL);
		this->minVoltage = (*this->sc)["KickHelper"]->get<double>("KickConfiguration", "MinVoltage", NULL);
		this->maxVoltage = (*this->sc)["KickHelper"]->get<double>("KickConfiguration", "MaxVoltage", NULL);
		this->powerMult = (*this->sc)["KickHelper"]->get<double>("KickConfiguration", "Multiplier", NULL);
		this->preciseShotMaxDistance = (*this->sc)["KickHelper"]->get<double>("PreciseShot", "MaxDistance", NULL);
		this->preciseShotMinDistance = (*this->sc)["KickHelper"]->get<double>("PreciseShot", "MinDistance", NULL);
		this->preciseShotMaxTolerance = (*this->sc)["KickHelper"]->get<double>("PreciseShot", "MaxTolerance", NULL);
		this->shortPassPower = (*this->sc)["KickHelper"]->get<double>("StandardPass", "ShortPassPower", NULL);
		this->kickerAngleOffset = (*this->sc)["KickHelper"]->get<double>("KickConfiguration", "KickerAngleOffset", NULL);
		this->kickerAngle = kickerAngle + (kickerAngleOffset / M_PI * 180);
		kick2GoalCurve = nullptr;
		kickHighPass = nullptr;
		kickLowPass = nullptr;
		init();
		lowShovelSelected = false;
		this->kickerVoltage = 0;
	}

	Kicker::~Kicker()
	{
		delete kick2GoalCurve;
		delete kickHighPass;
		delete kickLowPass;
	}

	bool Kicker::init()
	{
		kick2GoalCurve = new KickCurve("Kick2GoalCurve");
		kickHighPass = new KickCurve("HighPassCurve");
		kickLowPass = new KickCurve("LowPassCurve");

		return true;
	}

	float Kicker::getKickerVoltage(){
		return this->kickerVoltage;
	}

	void Kicker::setKickerVoltage(float kickerVoltage) {
		this->kickerVoltage = kickerVoltage;
	}

	int Kicker::getKickPowerPass(double dist)
	{
		if (lowShovelSelected)
		{
			return kickLowPass->getKickPower(dist, 0);
		}
		else
		{
			return kickHighPass->getKickPower(dist, 0);
		}
	}

	int Kicker::getKickPowerSlowPass(double dist)
	{
		return shortPassPower;
	}

	int Kicker::getKickerCount()
	{
		return kickerCount;
	}

	int Kicker::getKickPower(double dist, double height, double velo)
	{
		return kick2GoalCurve->getKickPower(dist, velo);
	}

	bool Kicker::mayShoot()
	{
		if (!wm->game->isMayScore())
		{
			return false;
		}
		//DistanceScan ds = wm.DistanceScan;
		//if (ds == null) return false;
		shared_ptr<vector<msl_sensor_msgs::ObstacleInfo>> obs = wm->obstacles->getEgoVisionObstacles();
		if (obs == nullptr)
		{
			return false;
		}
		shared_ptr<geometry::CNPoint2D> ballPos = wm->ball->getEgoBallPosition();
		if (ballPos == nullptr || !wm->ball->haveBall())
		{
			return false;
		}
		double dang = kickerAngle;
		for (int i = 0; i < obs->size(); i++)
		{
			if (make_shared<geometry::CNPoint2D>(obs->at(i).x, obs->at(i).y)->length() > 550)
				continue;
			if (abs(geometry::deltaAngle(make_shared<geometry::CNPoint2D>(obs->at(i).x, obs->at(i).y)->angleTo(), dang)) < 15.0 * M_PI / 180.0)
			{
				return false;
			}
		}
		return true;
	}

	shared_ptr<geometry::CNPoint2D> Kicker::getFreeGoalVector()
	{
		auto ownPos = wm->rawSensorData->getOwnPositionVision();
		auto dstscan = wm->rawSensorData->getDistanceScan();
		if (ownPos == nullptr || dstscan == nullptr)
		{
			return nullptr;
		}
		validGoalPoints.clear();
		double x = wm->field->getFieldLength() / 2;
		double y = -1000 + preciseShotMaxTolerance;
		shared_ptr<geometry::CNPoint2D> aim = make_shared<geometry::CNPoint2D>(x, y);
		double samplePoints = 4;

		for (double i = 0.0; i < samplePoints; i += 1.0)
		{
			shared_ptr<geometry::CNPoint2D> egoAim = aim->alloToEgo(*ownPos);
			double dist = egoAim->length();
			double opDist = minFree(egoAim->angleTo(), 200, dstscan);
			if (opDist > 1000 && (opDist >= dist || abs(opDist - dist) > 1500))
			{
				validGoalPoints.push_back(egoAim);
			}
			aim->y += 2 * abs(y) / samplePoints;
		}
		if (validGoalPoints.size() > 0)
		{
			shared_ptr<geometry::CNPoint2D> ret = nullptr;
			double max = numeric_limits<double>::min();
			for (int i = 0; i < validGoalPoints.size(); i++)
			{
				if (validGoalPoints[i]->length() > max)
				{
					max = validGoalPoints[i]->length();
					ret = validGoalPoints[i];
				}
			}
			return ret;
		}
		else
		{
			return nullptr;
		}
	}

	double Kicker::minFree(double angle, double width, shared_ptr<vector<double>> dstscan)
	{
		double sectorWidth = 2.0 * M_PI / dstscan->size();
		int startSector = mod((int)floor(angle / sectorWidth), dstscan->size());
		double minfree = dstscan->at(startSector);
		double dist, dangle;
		for (int i = 1; i < dstscan->size() / 4; i++)
		{
			dist = dstscan->at(mod((startSector + i), dstscan->size()));
			dangle = sectorWidth * i;
			if (abs(dist * sin(dangle)) < width)
			{
				minfree = min(minfree, abs(dist * cos(dangle)));
			}

			dist = dstscan->at(mod((startSector - i), dstscan->size()));
			if (abs(dist * sin(dangle)) < width)
			{
				minfree = min(minfree, abs(dist * cos(dangle)));
			}

		}
		return minfree;
	}

	double Kicker::getPassKickpower(double dist, double arrivalTime)
	{
		double refDist = 4460.0;

		// some savety conditions
		if (arrivalTime < 0.1)
		{
			arrivalTime = 0.1;
		}
		if (dist < 500.0)
		{
			dist = 500.0;
		}

		// scale because the function is fitted for 4.46m
		arrivalTime = arrivalTime * (refDist / dist);

		// coefficients
		double a = 3.9536753147006084E+02;
		double b = 7.6949794509362546E-01;

		// calc kickpower
		double kickPower = a * exp(b / arrivalTime);

		// min max kickpower constraints
		if (kickPower < 450.0)
		{
			kickPower = 450.0;
		}
		if (kickPower > 2000.0)
		{
			kickPower = 2000.0;
		}
		return kickPower;
	}

	double Kicker::getPassVelocity(double kickpower)
	{
		double a = 2.2592655783923168E-01;
		double b = -1.7011763554649019E+03;

		double temp = 0.0;
		temp += a * sqrt(kickpower);
		temp += b / kickpower;
		return temp * 1000.0;
	}

	/**
	 * This method is primarily for kicking a lob shot over a robot. So it will return kick powers between 1200 and 2800.
	 *
	 * If it returns -1 the tolerance in height is not matched.
	 *
	 * @param dist is the distance in mm
	 * @param height is the height of the center of the ball in mm
	 * @param heightTolerance is the tolerance for the height in mm (30.0mm is the minimum tolerance)
	 */
	double Kicker::getKickPowerForLobShot(double dist, double height, double heightTolerance )
	{
		// scale to meter :)
		dist = dist/1000.0;
		height = (height-120)/1000.0;
		heightTolerance = heightTolerance/1000.0;

		double g = 9.81;
		double vSample = 6.5;
		double vOptimal = 0;
		double heightErr = 100000.0;

		for (int i = 0; i < 143; i++)
		{
			double initialShootAngle = 2.676119513 * vSample + 12.70950743;
			initialShootAngle *=  M_PI / 180;
			double y = dist * tan(initialShootAngle)
					- (g * dist * dist) / (2 * vSample * vSample * cos(initialShootAngle) * cos(initialShootAngle));
			double curHeightErr = abs(height - y);
			if (curHeightErr < heightErr)
			{
				heightErr = curHeightErr;
				vOptimal = vSample;
			}
			else
			{
				break;
			}

			vSample += 0.035; // 100 steps until 11,5m/s
		}

		if(heightErr > heightTolerance)
		{
			cout << "Kicker: HeightErr: " << heightErr << endl;
			return -1;
		}

		// function to map v0 to kickPower
		double f2 = 350.0;
		double f3 = 11.5;
		double f4 = 850.0;

		double kickPower = (-log(1 - (vOptimal/f3)) * f2) + f4;

		return kickPower;
	}

	double Kicker::getPreciseShotMaxDistance()
	{
		return preciseShotMaxDistance;
	}

	double Kicker::getPreciseShotMaxTolerance()
	{
		return preciseShotMaxTolerance;
	}

	double Kicker::getPreciseShotMinDistance()
	{
		return preciseShotMinDistance;
	}

	int Kicker::getShortPassPower()
	{
		return shortPassPower;
	}

	int Kicker::mod(int x, int y)
	{
		int z = x % y;
		if (z < 0)
		{
			return y + z;
		}
		else
		{
			return z;
		}
	}

	void Kicker::processKickConstrolMsg(msl_actuator_msgs::KickControl& km)
	{
		shared_ptr<msl_actuator_msgs::KickControl> cmd = make_shared<msl_actuator_msgs::KickControl>();

		*cmd = km;
		shared_ptr<InformationElement<msl_actuator_msgs::KickControl>> jcmd = make_shared<
				InformationElement<msl_actuator_msgs::KickControl>>(cmd, wm->getTime());
		jcmd->certainty = 1.0;
		kickControlMsgs.add(jcmd);
	}

	shared_ptr<msl_actuator_msgs::KickControl> Kicker::getKickConstrolMsg(int index) {
		auto x = kickControlMsgs.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > 1000000000)
		{
			return nullptr;
		}
		return x->getInformation();
	}



} /* namespace msl */
