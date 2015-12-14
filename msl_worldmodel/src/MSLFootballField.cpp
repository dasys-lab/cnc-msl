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

	MSLFootballField * MSLFootballField::getInstance()
	{

		if (instance == NULL)
		{
			instance = new MSLFootballField();
		}

		return instance;

	}

	bool MSLFootballField::isInsideField(shared_ptr<geometry::CNPoint2D> point, double tolerance)
	{
		return abs(point->x) < FieldLength / 2 + tolerance && abs(point->y) < FieldWidth / 2 + tolerance;
	}

	bool MSLFootballField::isInsideOwnPenalty(shared_ptr<geometry::CNPoint2D> p, double tolerance)
	{
		return p->x - tolerance < -FieldLength / 2.0 + PenaltyAreaLength && abs(p->y) - tolerance < PenaltyAreaWidth / 2.0;
	}

	bool MSLFootballField::isInsideEnemyPenalty(shared_ptr<geometry::CNPoint2D> p, double tolerance)
	{
		return p->x + tolerance > FieldLength / 2.0 - PenaltyAreaLength && abs(p->y) - tolerance < PenaltyAreaWidth / 2.0;
	}

	bool MSLFootballField::isInsidePenalty(shared_ptr<geometry::CNPoint2D> p, double tolerance)
	{
		return isInsideOwnPenalty(p, tolerance) || isInsideEnemyPenalty(p, tolerance);
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

