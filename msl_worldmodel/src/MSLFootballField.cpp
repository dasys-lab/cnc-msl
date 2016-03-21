#include <MSLFootballField.h>
#include <iostream>

namespace msl
{

	double MSLFootballField::FieldLength = 11200.0;
	double MSLFootballField::FieldWidth = 8000.0;
	double MSLFootballField::PenaltyAreaWidth = 1200.0;
	double MSLFootballField::PenaltyAreaLength = 4000.0;
	double MSLFootballField::GoalAreaWidth = 3000.0;
	double MSLFootballField::GoalAreaLength = 700.0;
	double MSLFootballField::CornerCircleRadius = 350.0;
	double MSLFootballField::MiddleCircleRadius = 1000.0;
	double MSLFootballField::LineWidth = 75.0;
	double MSLFootballField::GoalWidth = 2000.0;
	double MSLFootballField::PenaltySpot = 3000.0;
	double MSLFootballField::Surrounding = 1500.0;
	bool MSLFootballField::GoalInnerAreaExists = false;
	bool MSLFootballField::CornerCircleExists = false;
	double MSLFootballField::PenaltyAreaMappingTolerance = 300;
	double MSLFootballField::MaxDistanceSqr = 12000*12000+18000*18000;
	double MSLFootballField::MaxDistance = sqrt(12000*12000+18000*18000);
	MSLFootballField * MSLFootballField::instance = NULL;

	MSLFootballField::MSLFootballField()
	{

		this->sc = SystemConfig::getInstance();

		FieldLength = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "FieldLength", NULL);
		FieldWidth = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "FieldWidth", NULL);
		PenaltyAreaLength = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "PenaltyAreaXSize", NULL);
		PenaltyAreaWidth = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "PenaltyAreaYSize", NULL);
		GoalAreaLength = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "GoalAreaXSize", NULL);
		GoalAreaWidth = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "GoalAreaYSize", NULL);
		CornerCircleRadius = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "CornerCircleRadius",
		NULL);
		LineWidth = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "LineWidth", NULL);
		GoalWidth = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "GoalWidth", NULL);
		GoalInnerAreaExists = (*this->sc)["Globals"]->get<bool>("Globals", "FootballField", "GoalInnerAreaExists",
		NULL);
		CornerCircleExists = (*this->sc)["Globals"]->get<bool>("Globals", "FootballField", "CornerCircleExists", NULL);
		PenaltySpot = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "PenaltySpot", NULL);
		Surrounding = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "Surrounding", NULL);
		PenaltyAreaMappingTolerance = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "PenaltyAreaMappingTolerance", NULL);

		MaxDistanceSqr = FieldLength*FieldLength + FieldWidth*FieldWidth;
		MaxDistance = sqrt(MaxDistanceSqr);

		std::cout << "MSLFootballField::FieldLength = " << FieldLength << std::endl;
		std::cout << "MSLFootballField::FieldWidth = " << FieldWidth << std::endl;
		std::cout << "MSLFootballField::PenaltyAreaLength = " << PenaltyAreaLength << std::endl;
		std::cout << "MSLFootballField::PenaltyAreaWidth = " << PenaltyAreaWidth << std::endl;
		std::cout << "MSLFootballField::MiddleCircleRadius = " << MiddleCircleRadius << std::endl;
		std::cout << "MSLFootballField::GoalAreaLength = " << GoalAreaLength << std::endl;
		std::cout << "MSLFootballField::GoalAreaWidth = " << GoalAreaWidth << std::endl;
		std::cout << "MSLFootballField::CornerCircleRadius = " << CornerCircleRadius << std::endl;
		std::cout << "MSLFootballField::LineWidth = " << LineWidth << std::endl;
		std::cout << "MSLFootballField::GoalInnerAreaExists = " << GoalInnerAreaExists << std::endl;
		std::cout << "MSLFootballField::CornerCircleExists = " << CornerCircleExists << std::endl;

	}

	MSLFootballField::~MSLFootballField()
	{

	}

	MSLFootballField* MSLFootballField::getInstance()
	{
		static MSLFootballField instance;
		return &instance;
	}

	bool MSLFootballField::isInsideField(shared_ptr<geometry::CNPoint2D> point, double tolerance)
	{
		return abs(point->x) < FieldLength / 2 + tolerance && abs(point->y) < FieldWidth / 2 + tolerance;
	}

	bool MSLFootballField::isInsideOwnPenalty(shared_ptr<geometry::CNPoint2D> p, double tolerance)
	{
		return p->x - tolerance < -FieldLength / 2.0 + PenaltyAreaLength
				&& abs(p->y) - tolerance < PenaltyAreaWidth / 2.0;
	}

	bool MSLFootballField::isInsideEnemyPenalty(shared_ptr<geometry::CNPoint2D> p, double tolerance)
	{
		return p->x + tolerance > FieldLength / 2.0 - PenaltyAreaLength
				&& abs(p->y) - tolerance < PenaltyAreaWidth / 2.0;
	}

	bool MSLFootballField::isInsidePenalty(shared_ptr<geometry::CNPoint2D> p, double tolerance)
	{
		return isInsideOwnPenalty(p, tolerance) || isInsideEnemyPenalty(p, tolerance);
	}

	/// <summary>Checks whether a given point is inside the field</summary>
	bool MSLFootballField::isInsideField(shared_ptr<geometry::CNPoint2D> p)
	{
		return abs(p->x) < FieldLength / 2 && abs(p->y) < FieldWidth / 2;
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::mapOutOfOwnPenalty(shared_ptr<geometry::CNPoint2D> inp)
	{
		double tolerance = PenaltyAreaMappingTolerance;
		if (!isInsideOwnPenalty(inp, tolerance))
		{
			return inp;
		}
		//compute vector to closest point on penalty line:
		double xline = -FieldLength / 2.0 + PenaltyAreaLength + tolerance;
		double dist = abs(inp->x - xline);
		double yline = PenaltyAreaWidth / 2.0 + tolerance;
		if (inp->y > 0)
		{
			if (dist > yline - inp->y)
			{
				return make_shared<geometry::CNPoint2D>(inp->x, yline);
			}
			else
			{
				return make_shared<geometry::CNPoint2D>(xline, inp->y);
			}
		}
		else
		{
			if (dist > yline + inp->y)
			{
				return make_shared<geometry::CNPoint2D>(inp->x, -yline);
			}
			else
			{
				return make_shared<geometry::CNPoint2D>(xline, inp->y);
			}
		}
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::mapOutOfOwnPenalty(shared_ptr<geometry::CNPoint2D> inp,
																			shared_ptr<geometry::CNPoint2D> alongVec)
	{
		double tolerance = PenaltyAreaMappingTolerance;
		if (!isInsideOwnPenalty(inp, tolerance))
		{
			return inp;
		}
		double xline = -FieldLength / 2.0 + PenaltyAreaLength + tolerance;
		double yline = PenaltyAreaWidth / 2.0 + tolerance;
		if (alongVec->y == 0.0)
		{
			return make_shared<geometry::CNPoint2D>(xline, inp->y);
		}
		shared_ptr<geometry::CNPoint2D> cur = make_shared<geometry::CNPoint2D>(inp->x, inp->y);
		alongVec = alongVec->normalize();
		double d;
		double dist = 1000000;
		//Left Line: (-1,0)
		if (alongVec->y < 0)
		{
			d = (-yline - inp->y) / alongVec->y;
			cur->x = alongVec->x * d + inp->x;
			cur->y = -yline;
			dist = abs(d);
		}
		else
		{
			//Right Line: (1,0)
			d = (yline - inp->y) / alongVec->y;
			dist = abs(d);
			cur->x = alongVec->x * d + inp->x;
			cur->y = yline;
		}
		if (alongVec->x == 0)
		{
			return cur;
		}
		//Top Line: (0,1)
		d = (xline - inp->x) / alongVec->x;
		if (abs(d) < dist)
		{
			cur->x = xline;
			cur->y = alongVec->y * d + inp->y;
		}
		return cur;

	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::mapOutOfPenalty(shared_ptr<geometry::CNPoint2D> inp)
	{
		return mapOutOfOwnPenalty(mapOutOfEnemyPenalty(inp));
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::mapOutOfEnemyPenalty(shared_ptr<geometry::CNPoint2D> inp)
	{
		double tolerance = PenaltyAreaMappingTolerance;
		if (!isInsideEnemyPenalty(inp, tolerance))
		{
			return inp;
		}
		//compute vector to closest point on penalty line:
		double xline = FieldLength / 2.0 - PenaltyAreaLength - tolerance;
		double dist = abs(inp->x - xline);
		double yline = PenaltyAreaWidth / 2.0 + tolerance;
		if (inp->y > 0)
		{
			if (dist > yline - inp->y)
			{
				return make_shared<geometry::CNPoint2D>(inp->x, yline);
			}
			else
				return make_shared<geometry::CNPoint2D>(xline, inp->y);
		}
		else
		{
			if (dist > yline + inp->y)
			{
				return make_shared<geometry::CNPoint2D>(inp->x, -yline);
			}
			else
				return make_shared<geometry::CNPoint2D>(xline, inp->y);
		}
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::mapOutOfEnemyPenalty(shared_ptr<geometry::CNPoint2D> inp,
																			shared_ptr<geometry::CNPoint2D> alongVec)
	{
		double tolerance = PenaltyAreaMappingTolerance;
		if (!isInsideEnemyPenalty(inp, tolerance))
		{
			return inp;
		}

		double xline = FieldLength / 2.0 - PenaltyAreaLength - tolerance;
		double yline = PenaltyAreaWidth / 2.0 + tolerance;
		if (alongVec->y == 0.0)
		{
			return make_shared<geometry::CNPoint2D>(xline, inp->y);
		}
		shared_ptr<geometry::CNPoint2D> cur = make_shared<geometry::CNPoint2D>(inp->x, inp->y);
		alongVec = alongVec->normalize();
		double d;
		double dist = 1000000;
		//Left Line: (-1,0)
		if (alongVec->y < 0)
		{
			d = (-yline - inp->y) / alongVec->y;
			cur->x = alongVec->x * d + inp->x;
			cur->y = -yline;
			dist = abs(d);
		}
		else
		{
			//Right Line: (1,0)
			d = (yline - inp->y) / alongVec->y;
			dist = abs(d);
			cur->x = alongVec->x * d + inp->x;
			cur->y = yline;
		}
		if (alongVec->x == 0)
		{
			return cur;
		}
		//Top Line: (0,1)
		d = (xline - inp->x) / alongVec->x;
		if (abs(d) < dist)
		{
			cur->x = xline;
			cur->y = alongVec->y * d + inp->y;
		}
		return cur;
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::mapInsideField(shared_ptr<geometry::CNPoint2D> inp)
	{
		double tolerance = 150;
		if (isInsideField(inp))
		{
			return inp;
		}
		shared_ptr<geometry::CNPoint2D> ret = make_shared<geometry::CNPoint2D>();
		double xline = FieldLength / 2.0 + tolerance;
		double yline = FieldWidth / 2.0 + tolerance;
		if (inp->x < -xline)
		{
			ret->x = -xline;
		}
		else if (inp->x > xline)
		{
			ret->x = xline;
		}
		else
		{
			ret->x = inp->x;
		}
		if (inp->y < -yline)
		{
			ret->y = -yline;
		}
		else if (inp->y > yline)
		{
			ret->y = yline;
		}
		else
		{
			ret->y = inp->y;
		}
		return ret;
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::mapInsideField(shared_ptr<geometry::CNPoint2D> inp,
																		double tolerance)
	{
		if(isInsideField(inp, tolerance))
		{
			return inp;
		}
		shared_ptr<geometry::CNPoint2D> ret = make_shared<geometry::CNPoint2D>();
		double xline = FieldLength / 2.0 + tolerance;
		double yline = FieldWidth / 2.0 + tolerance;
		if (inp->x < -xline)
		{
			ret->x = -xline;
		}
		else if (inp->x > xline)
		{
			ret->x = xline;
		}
		else
		{
			ret->x = inp->x;
		}
		if (inp->y < -yline)
		{
			ret->y = -yline;
		}
		else if (inp->y > yline)
		{
			ret->y = yline;
		}
		else
		{
			ret->y = inp->y;
		}
		return ret;
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::mapInsideOwnPenaltyArea(shared_ptr<geometry::CNPoint2D> inp,
																				double tolerance)
	{
		double toleranceCheck = 0;
		//double tolerance = penaltyAreaMappingTolerance;
		if (isInsideOwnPenalty(inp, toleranceCheck))
		{
			return inp;
		}
		shared_ptr<geometry::CNPoint2D> ret = make_shared<geometry::CNPoint2D>();
		double xline = -FieldLength / 2.0 + PenaltyAreaLength - tolerance;
		double yline = PenaltyAreaWidth / 2.0 - tolerance;
		//if (inp.X < -xline) ret.X = -xline;
		if (inp->x > xline)
		{
			ret->x = xline;
		}
		else
		{
			ret->x = inp->x;
		}
		if (inp->y < -yline)
		{
			ret->y = -yline;
		}
		else if (inp->y > yline)
		{
			ret->y = yline;
		}
		else
		{
			ret->y = inp->y;
		}
		return ret;
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::mapInsideOwnPenaltyArea(shared_ptr<geometry::CNPoint2D> inp)
	{
		double toleranceCheck = 0;
		double tolerance = PenaltyAreaMappingTolerance;
		if (isInsideOwnPenalty(inp, toleranceCheck))
		{
			return inp;
		}
		shared_ptr<geometry::CNPoint2D> ret = make_shared<geometry::CNPoint2D>();
		double xline = -FieldLength / 2.0 + PenaltyAreaLength - tolerance;
		double yline = PenaltyAreaWidth / 2.0 - tolerance;
		//if (inp.X < -xline) ret.X = -xline;
		if (inp->x > xline)
		{
			ret->x = xline;
		}
		else
		{
			ret->x = inp->x;
		}
		if (inp->y < -yline)
		{
			ret->y = -yline;
		}
		else if (inp->y > yline)
		{
			ret->y = yline;
		}
		else
		{
			ret->y = inp->y;
		}
		return ret;
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::mapInsideField(shared_ptr<geometry::CNPoint2D> inp,
																		shared_ptr<geometry::CNPoint2D> alongVec)
	{
		double tolerance = 150;
		if (isInsideField(inp))
		{
			return inp;
		}
		double xline = FieldLength / 2.0 + tolerance;
		double yline = FieldWidth / 2.0 + tolerance;

		double d = 0;
		shared_ptr<geometry::CNPoint2D> cur = make_shared<geometry::CNPoint2D>(inp->x, inp->y);
		if (cur->x < -xline)
		{
			if (alongVec->x != 0.0)
			{
				d = alongVec->y / alongVec->x;
				cur->y = (-xline - cur->x) * d;
			}
			else
			{
				//ignore bad vector
			}
			cur->x = -xline;
		}
		else if (cur->x > xline)
		{
			if (alongVec->x != 0.0)
			{
				d = alongVec->y / alongVec->x;
				cur->y = (xline - cur->x) * d;
			}
			else
			{
				//ignore bad vector
			}
			cur->x = xline;
		}
		if (cur->y < -yline)
		{
			if (alongVec->x != 0.0)
			{
				d = alongVec->x / alongVec->y;
				cur->x = (-yline - cur->y) * d;
			}
			else
			{
				//ignore bad vector
			}
			cur->y = -yline;
		}
		else if (cur->y > yline)
		{
			if (alongVec->x != 0.0)
			{
				d = alongVec->x / alongVec->y;
				cur->x = (yline - cur->y) * d;
			}
			else
			{
				//ignore bad vector
			}
			cur->y = yline;
		}
		return cur;
	}

	bool MSLFootballField::isInsideOwnKeeperArea(shared_ptr<geometry::CNPoint2D> p, double tolerance)
	{
		return p->x - tolerance < -FieldLength / 2.0 + GoalAreaLength
				&& abs(p->y) - tolerance < GoalAreaWidth / 2.0;
	}

	bool MSLFootballField::isInsideEnemyKeeperArea(shared_ptr<geometry::CNPoint2D> p, double tolerance)
	{
		return p->x + tolerance > FieldLength / 2.0 - GoalAreaLength
				&& abs(p->y) - tolerance < GoalAreaWidth / 2.0;
	}

	bool MSLFootballField::isInsideKeeperArea(shared_ptr<geometry::CNPoint2D> p, double tolerance)
	{
		return isInsideOwnKeeperArea(p, tolerance) || isInsideEnemyKeeperArea(p, tolerance);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::mapOutOfOwnKeeperArea(shared_ptr<geometry::CNPoint2D> inp)
	{
		double tolerance = 150;
		if (!isInsideOwnKeeperArea(inp, tolerance))
		{
			return inp;
		}
		//compute vector to closest point on penalty line:
		double xline = -FieldLength / 2.0 + GoalAreaLength + tolerance;
		double dist = abs(inp->x - xline);
		double yline = GoalAreaWidth / 2.0 + tolerance;
		if (inp->y > 0)
		{
			if (dist > yline - inp->y)
			{
				return make_shared<geometry::CNPoint2D>(inp->x, yline);
			}
			else
				return make_shared<geometry::CNPoint2D>(xline, inp->y);
		}
		else
		{
			if (dist > yline + inp->y)
			{
				return make_shared<geometry::CNPoint2D>(inp->x, -yline);
			}
			else
				return make_shared<geometry::CNPoint2D>(xline, inp->y);
		}
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::mapOutOfEnemyKeeperArea(shared_ptr<geometry::CNPoint2D> inp)
	{
		double tolerance = 250;
		if (!isInsideEnemyKeeperArea(inp, tolerance))
		{
			return inp;
		}
		//compute vector to closest point on penalty line:
		double xline = FieldLength / 2.0 - GoalAreaLength - tolerance;
		double dist = abs(inp->x - xline);
		double yline = GoalAreaWidth / 2.0 + tolerance;
		if (inp->y > 0)
		{
			if (dist > yline - inp->y)
			{
				return make_shared<geometry::CNPoint2D>(inp->x, yline);
			}
			else
				return make_shared<geometry::CNPoint2D>(xline, inp->y);
		}
		else
		{
			if (dist > yline + inp->y)
			{
				return make_shared<geometry::CNPoint2D>(inp->x, -yline);
			}
			else
				return make_shared<geometry::CNPoint2D>(xline, inp->y);
		}
	}

	/// <summary>
	/// Computes a point outside own penalty on the line from 'from' to 'to', given 'from' is outside
	/// </summary>
	shared_ptr<geometry::CNPoint2D> MSLFootballField::keepOutOfOwnPenalty(shared_ptr<geometry::CNPoint2D> from,
																			shared_ptr<geometry::CNPoint2D> to)
	{
		double tolerance = 100;
		if (!isInsideOwnPenalty(to, tolerance))
		{
			return to;
		}
		if (from->distanceTo(to) < 50)
		{
			return mapOutOfOwnPenalty(to);
		}
		shared_ptr<geometry::CNPoint2D> vec = to - from;
		//compute intersection with fron line:
		double xline = -FieldLength / 2.0 + PenaltyAreaLength + tolerance;
		double yline = PenaltyAreaWidth / 2.0 + tolerance;
		double u, y;
		if (vec->x != 0)
		{
			u = (xline - from->x) / vec->x;
			y = from->y + u * vec->y;
			if (abs(y) < yline)
			{
				return make_shared<geometry::CNPoint2D>(xline, y);
			}
			else
			{
				if (y > 0)
				{
					u = (yline - from->y) / vec->y; //vec.Y cannot be 0
					return make_shared<geometry::CNPoint2D>(from->x + u * vec->x, yline);
				}
				else
				{
					u = (yline - from->y) / vec->y;
					return make_shared<geometry::CNPoint2D>(from->x + u * vec->x, -yline);
				}
			}
		}
		else
		{ //vec.X == 0
			if (from->y > 0)
			{
				return make_shared<geometry::CNPoint2D>(to->x, yline);
			}
			else
			{
				return make_shared<geometry::CNPoint2D>(to->x, -yline);
			}
		}
	}

	/// <summary>
	/// Computes the distance to the fringe of the field from a point towards a direction given as angle
	/// </summary>

	double MSLFootballField::distanceToLine(shared_ptr<geometry::CNPoint2D> from, double angle)
	{
		return distanceToLine(from, angle, 0);
	}

	double MSLFootballField::distanceToLine(shared_ptr<geometry::CNPoint2D> from, double angle, double extendFieldLines)
	{
		double distance = 100000;
		double vx = cos(angle);
		double vy = sin(angle);
		double d;
		double x = FieldLength / 2 + extendFieldLines;
		double y = FieldWidth / 2 + extendFieldLines;
		//Left Line: (1,0)
		d = (y - from->y) / vy;
		if (d > 0)
			distance = min(distance, d);
		//Right Line: (1,0)
		d = (-y - from->y) / vy;
		if (d > 0)
			distance = min(distance, d);
		//Top Line: (0,1)
		d = (x - from->x) / vx;
		if (d > 0)
			distance = min(distance, d);
		//Bot Line: (0,1)
		d = (-x - from->x) / vx;
		if (d > 0)
			distance = min(distance, d);
		return distance;

	}

	double MSLFootballField::projectVectorOntoX(shared_ptr<geometry::CNPoint2D> origin,
												shared_ptr<geometry::CNPoint2D> dir, double x)
	{
		// origin.x + dir.x *t = x =>
		// t = (x-origin.x)/dir.x;
		// y= origin.y+dir.y*(x-origin.x)/dir.x;
		return origin->y + dir->y * (x - origin->x) / dir->x;
	}

	double MSLFootballField::projectVectorOntoY(shared_ptr<geometry::CNPoint2D> origin,
												shared_ptr<geometry::CNPoint2D> dir, double y)
	{
		// origin.y + dir.y *t = y =>
		// t = (y-origin.y)/dir.y;
		// x= origin.x+dir.x*(y-origin.y)/dir.y;
		return origin->x + dir->x * (y - origin->y) / dir->y;
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posCenterMarker()
	{
		return make_shared<geometry::CNPoint2D>(0, 0);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLeftOwnCorner()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2, FieldWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posRightOwnCorner()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2, -FieldWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLeftOppCorner()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2, FieldWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posRightOppCorner()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2, -FieldWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLeftOwnGoalPost()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2, GoalWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posRightOwnGoalPost()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2, -GoalWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLeftOppGoalPost()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2, GoalWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posRightOppGoalPost()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2, -GoalWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posOwnPenaltyMarker()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2 + PenaltySpot, 0.0);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posRightOwnRestartMarker()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2 + PenaltySpot, -FieldWidth / 4);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLeftOwnRestartMarker()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2 + PenaltySpot, FieldWidth / 4);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posOppPenaltyMarker()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2 - PenaltySpot, 0.0);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posRightOppRestartMarker()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2 - PenaltySpot, -FieldWidth / 4);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLeftOppRestartMarker()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2 - PenaltySpot, FieldWidth / 4);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posOppGoalMid()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2, 0);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posOwnGoalMid()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2, 0);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLROppHalf()
	{
		return make_shared<geometry::CNPoint2D>(0.0, -FieldWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posULOwnHalf()
	{
		return make_shared<geometry::CNPoint2D>(0.0, FieldWidth / 2);
	}

	// TODO calculate penalty stuff with right parameters from globals.conf
	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLROwnPenaltyArea()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2, -PenaltyAreaWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posULOwnPenaltyArea()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2 + PenaltyAreaLength, PenaltyAreaWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLROppPenaltyArea()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2 - PenaltyAreaLength, -PenaltyAreaWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posULOppPenaltyArea()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2, PenaltyAreaWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLROwnGoalArea()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2, -GoalAreaWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posULOwnGoalArea()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2 + GoalAreaLength, GoalAreaWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLROppGoalArea()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2 - GoalAreaLength, -GoalAreaWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posULOppGoalArea()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2, GoalAreaWidth / 2);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLeftRestartMarker()
	{
		return make_shared<geometry::CNPoint2D>(0.0, FieldWidth / 4);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posRightRestartMarker()
	{
		return make_shared<geometry::CNPoint2D>(0.0, -FieldWidth / 4);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posLRSurrounding()
	{
		return make_shared<geometry::CNPoint2D>(-FieldLength / 2 - Surrounding, -FieldWidth / 2 + Surrounding);
	}

	shared_ptr<geometry::CNPoint2D> MSLFootballField::posULSurrounding()
	{
		return make_shared<geometry::CNPoint2D>(FieldLength / 2 + Surrounding, FieldWidth / 2 - Surrounding);
	}
}

