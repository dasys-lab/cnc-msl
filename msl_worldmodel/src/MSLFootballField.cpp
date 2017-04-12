#include "MSLFootballField.h"

#include "MSLWorldModel.h"
#include <cnc_geometry/CNVecAllo.h>

#include <SystemConfig.h>
#include <iostream>

namespace msl
{
MSLFootballField::MSLFootballField(MSLWorldModel *wm)
{
    this->sc = supplementary::SystemConfig::getInstance();
    this->wm = wm;
    this->CurrentField = (*this->sc)["FootballField"]->get<string>("FootballField", "CurrentField", NULL);
    MiddleCircleRadius = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "MiddleCircleRadius", NULL);
    FieldLength = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "FieldLength", NULL);
    FieldWidth = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "FieldWidth", NULL);
    PenaltyAreaLength = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "PenaltyAreaXSize", NULL);
    PenaltyAreaWidth = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "PenaltyAreaYSize", NULL);
    GoalAreaLength = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "GoalAreaXSize", NULL);
    GoalAreaWidth = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "GoalAreaYSize", NULL);
    CornerCircleRadius = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "CornerCircleRadius", NULL);
    LineWidth = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "LineWidth", NULL);
    GoalWidth = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "GoalWidth", NULL);
    GoalInnerAreaExists = (*this->sc)["FootballField"]->get<bool>("FootballField", CurrentField.c_str(), "GoalInnerAreaExists", NULL);
    CornerCircleExists = (*this->sc)["FootballField"]->get<bool>("FootballField", CurrentField.c_str(), "CornerCircleExists", NULL);
    PenaltySpot = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "PenaltySpot", NULL);
    Surrounding = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "Surrounding", NULL);
    PenaltyAreaMappingTolerance = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "PenaltyAreaMappingTolerance", NULL);

    MaxDistanceSqr = FieldLength * FieldLength + FieldWidth * FieldWidth;
    MaxDistance = sqrt(MaxDistanceSqr);

    std::cout << "MSLFootballField::currentField = " << CurrentField << std::endl;
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

/// <summary>Checks whether a given point is inside the field</summary>
bool MSLFootballField::isInsideField(geometry::CNPointAllo point, double tolerance)
{
    return abs(point.x) < FieldLength / 2 + tolerance && abs(point.y) < FieldWidth / 2 + tolerance;
}

bool MSLFootballField::isInsideField(double x, double y, double tolerance)
{
    return abs(x) < FieldLength / 2 + tolerance && abs(y) < FieldWidth / 2 + tolerance;
}

bool MSLFootballField::isInsideOwnPenalty(const geometry::CNPointAllo p, double tolerance) const
{
    return p.x - tolerance < -FieldLength / 2.0 + PenaltyAreaLength && abs(p.y) - tolerance < PenaltyAreaWidth / 2.0;
}

bool MSLFootballField::isInsideOppPenalty(const geometry::CNPointAllo p, double tolerance) const
{
    return p.x + tolerance > FieldLength / 2.0 - PenaltyAreaLength && abs(p.y) - tolerance < PenaltyAreaWidth / 2.0;
}

bool MSLFootballField::isInsidePenalty(const geometry::CNPointAllo p, double tolerance) const
{
    return isInsideOwnPenalty(p, tolerance) || isInsideOppPenalty(p, tolerance);
}

geometry::CNPointAllo MSLFootballField::mapOutOfOwnPenalty(const geometry::CNPointAllo inp)
{
    if (!isInsideOwnPenalty(inp, PenaltyAreaMappingTolerance))
    {
        return inp;
    }
    // compute vector to closest point on penalty line:
    return mapOutsideArea(inp, -FieldLength / 2.0 + PenaltyAreaLength + PenaltyAreaMappingTolerance, PenaltyAreaWidth / 2.0 + PenaltyAreaMappingTolerance);
}

geometry::CNPointAllo MSLFootballField::mapOutOfOwnPenalty(geometry::CNPointAllo inp, geometry::CNVecAllo alongVec)
{
    double tolerance = PenaltyAreaMappingTolerance;
    if (!isInsideOwnPenalty(inp, tolerance))
    {
        return inp;
    }
    double xline = -FieldLength / 2.0 + PenaltyAreaLength + tolerance;
    double yline = PenaltyAreaWidth / 2.0 + tolerance;
    if (alongVec.y == 0.0)
    {
        return geometry::CNPointAllo(xline, inp.y);
    }
    geometry::CNPointAllo cur = geometry::CNPointAllo(inp.x, inp.y);
    alongVec = alongVec.normalize();
    double d;
    double dist = 1000000;
    // Left Line: (-1,0)
    if (alongVec.y < 0)
    {
        d = (-yline - inp.y) / alongVec.y;
        cur.x = alongVec.x * d + inp.x;
        cur.y = -yline;
        dist = abs(d);
    }
    else
    {
        // Right Line: (1,0)
        d = (yline - inp.y) / alongVec.y;
        dist = abs(d);
        cur.x = alongVec.x * d + inp.x;
        cur.y = yline;
    }
    if (alongVec.x == 0)
    {
        return cur;
    }
    // Top Line: (0,1)
    d = (xline - inp.x) / alongVec.x;
    if (abs(d) < dist)
    {
        cur.x = xline;
        cur.y = alongVec.y * d + inp.y;
    }
    return cur;
}

geometry::CNPointAllo MSLFootballField::mapOutsideArea(geometry::CNPointAllo inp, double xline, double yline)
{
    double xDist = abs(inp.x - xline);
    double yDist = yline - abs(inp.y);

    if (xDist > yDist)
    {
        if (inp.y > 0)
        {
            return geometry::CNPointAllo(inp.x, yline);
        }
        else
        {
            return geometry::CNPointAllo(inp.x, -yline);
        }
    }
    else
    {
        return geometry::CNPointAllo(xline, inp.y);
    }
}

geometry::CNPointAllo MSLFootballField::mapOutOfPenalty(geometry::CNPointAllo inp)
{
    return mapOutOfOwnPenalty(mapOutOfOppPenalty(inp));
}

geometry::CNPointAllo MSLFootballField::mapOutOfOppPenalty(geometry::CNPointAllo inp)
{
    double tolerance = PenaltyAreaMappingTolerance;
    if (!isInsideOppPenalty(inp, tolerance))
    {
        return inp;
    }
    // compute vector to closest point on penalty line:
    return mapOutsideArea(inp, FieldLength / 2.0 - PenaltyAreaLength - tolerance, PenaltyAreaWidth / 2.0 + tolerance);
}

geometry::CNPointAllo MSLFootballField::mapOutOfOppPenalty(geometry::CNPointAllo inp, geometry::CNVecAllo alongVec)
{
    double tolerance = PenaltyAreaMappingTolerance;
    if (!isInsideOppPenalty(inp, tolerance))
    {
        return inp;
    }

    double xline = FieldLength / 2.0 - PenaltyAreaLength - tolerance;
    double yline = PenaltyAreaWidth / 2.0 + tolerance;
    if (alongVec.y == 0.0)
    {
        return geometry::CNPointAllo(xline, inp.y);
    }
    geometry::CNPointAllo cur = geometry::CNPointAllo(inp.x, inp.y);
    alongVec = alongVec.normalize();
    double d;
    double dist = 1000000;
    // Left Line: (-1,0)
    if (alongVec.y < 0)
    {
        d = (-yline - inp.y) / alongVec.y;
        cur.x = alongVec.x * d + inp.x;
        cur.y = -yline;
        dist = abs(d);
    }
    else
    {
        // Right Line: (1,0)
        d = (yline - inp.y) / alongVec.y;
        dist = abs(d);
        cur.x = alongVec.x * d + inp.x;
        cur.y = yline;
    }
    if (alongVec.x == 0)
    {
        return cur;
    }
    // Top Line: (0,1)
    d = (xline - inp.x) / alongVec.x;
    if (abs(d) < dist)
    {
        cur.x = xline;
        cur.y = alongVec.y * d + inp.y;
    }
    return cur;
}

geometry::CNPointAllo MSLFootballField::mapInsideArea(geometry::CNPointAllo inp, double xline, double yline)
{
    return geometry::CNPointAllo(min(max(inp.x, -xline), xline), min(max(inp.y, -yline), yline));
}

geometry::CNPointAllo MSLFootballField::mapInsideField(geometry::CNPointAllo inp)
{
    return this->mapInsideField(inp, 150.0); // TODO config parameter
}

geometry::CNPointAllo MSLFootballField::mapInsideField(geometry::CNPointAllo inp, double tolerance)
{
    if (isInsideField(inp, tolerance))
    {
        return inp;
    }
    return mapInsideArea(inp, FieldLength / 2.0 + tolerance, FieldWidth / 2.0 + tolerance);
}

geometry::CNPointAllo MSLFootballField::mapInsideOwnPenaltyArea(geometry::CNPointAllo inp, double tolerance)
{
    if (isInsideOwnPenalty(inp, tolerance))
    {
        return inp;
    }
    return mapInsideArea(inp, -FieldLength / 2.0 + PenaltyAreaLength - tolerance, PenaltyAreaWidth / 2.0 - tolerance);
}

geometry::CNPointAllo MSLFootballField::mapInsideOwnPenaltyArea(geometry::CNPointAllo inp)
{
    double tolerance = PenaltyAreaMappingTolerance;
    if (isInsideOwnPenalty(inp, tolerance))
    {
        return inp;
    }
    return mapInsideArea(inp, -FieldLength / 2.0 + PenaltyAreaLength - tolerance, PenaltyAreaWidth / 2.0 - tolerance);
}

geometry::CNPointAllo MSLFootballField::mapInsideField(geometry::CNPointAllo inp, geometry::CNVecAllo alongVec)
{
    double tolerance = 150;
    if (isInsideField(inp))
    {
        return inp;
    }
    double xline = FieldLength / 2.0 + tolerance;
    double yline = FieldWidth / 2.0 + tolerance;

    double d = 0;
    geometry::CNPointAllo cur = geometry::CNPointAllo(inp.x, inp.y);
    if (cur.x < -xline)
    {
        if (alongVec.x != 0.0)
        {
            d = alongVec.y / alongVec.x;
            cur.y = (-xline - cur.x) * d;
        }
        else
        {
            // ignore bad vector
        }
        cur.x = -xline;
    }
    else if (cur.x > xline)
    {
        if (alongVec.x != 0.0)
        {
            d = alongVec.y / alongVec.x;
            cur.y = (xline - cur.x) * d;
        }
        else
        {
            // ignore bad vector
        }
        cur.x = xline;
    }
    if (cur.y < -yline)
    {
        if (alongVec.x != 0.0)
        {
            d = alongVec.x / alongVec.y;
            cur.x = (-yline - cur.y) * d;
        }
        else
        {
            // ignore bad vector
        }
        cur.y = -yline;
    }
    else if (cur.y > yline)
    {
        if (alongVec.x != 0.0)
        {
            d = alongVec.x / alongVec.y;
            cur.x = (yline - cur.y) * d;
        }
        else
        {
            // ignore bad vector
        }
        cur.y = yline;
    }
    return cur;
}

bool MSLFootballField::isInsideOwnGoalArea(geometry::CNPointAllo p, double tolerance)
{
    return p.x - tolerance < -FieldLength / 2.0 + GoalAreaLength && abs(p.y) - tolerance < GoalAreaWidth / 2.0;
}

bool MSLFootballField::isInsideOppGoalArea(geometry::CNPointAllo p, double tolerance)
{
    return p.x + tolerance > FieldLength / 2.0 - GoalAreaLength && abs(p.y) - tolerance < GoalAreaWidth / 2.0;
}

bool MSLFootballField::isInsideGoalArea(geometry::CNPointAllo p, double tolerance)
{
    return isInsideOwnGoalArea(p, tolerance) || isInsideOppGoalArea(p, tolerance);
}

geometry::CNPointAllo MSLFootballField::mapOutOfOwnGoalArea(geometry::CNPointAllo inp)
{
    double tolerance = 150;
    if (!isInsideOwnGoalArea(inp, tolerance))
    {
        return inp;
    }
    // compute vector to closest point on penalty line:
    return mapOutsideArea(inp, -FieldLength / 2.0 + GoalAreaLength + tolerance, GoalAreaWidth / 2.0 + tolerance);
}

geometry::CNPointAllo MSLFootballField::mapOutOfOppGoalArea(geometry::CNPointAllo inp)
{
    double tolerance = 250;
    if (!isInsideOppGoalArea(inp, tolerance))
    {
        return inp;
    }
    // compute vector to closest point on penalty line:
    return mapOutsideArea(inp, FieldLength / 2.0 - GoalAreaLength - tolerance, GoalAreaWidth / 2.0 + tolerance);
}

/**
 * Computes a point outside own penalty on the line from 'from' to 'to', given 'from' is outside
 */
geometry::CNPointAllo MSLFootballField::keepOutOfOwnPenalty(geometry::CNPointAllo from, geometry::CNPointAllo to)
{
    double tolerance = 100; // TODO: Make it a config parameter
    if (!isInsideOwnPenalty(to, tolerance))
    {
        return to;
    }
    if (from.distanceTo(to) < 50)
    {
        return mapOutOfOwnPenalty(to);
    }
    geometry::CNVecAllo vec = to - from;
    // compute intersection with front line:
    double xline = -FieldLength / 2.0 + PenaltyAreaLength + tolerance;
    double yline = PenaltyAreaWidth / 2.0 + tolerance;
    double u, y;
    if (vec.x != 0)
    {
        u = (xline - from.x) / vec.x;
        y = from.y + u * vec.y;
        if (abs(y) < yline)
        {
            return geometry::CNPointAllo(xline, y);
        }
        else
        {
            if (y > 0)
            {
                u = (yline - from.y) / vec.y; // vec.Y cannot be 0
                return geometry::CNPointAllo(from.x + u * vec.x, yline);
            }
            else
            {
                u = (yline - from.y) / vec.y;
                return geometry::CNPointAllo(from.x + u * vec.x, -yline);
            }
        }
    }
    else
    { // vec.X == 0
        if (from.y > 0)
        {
            return geometry::CNPointAllo(to.x, yline);
        }
        else
        {
            return geometry::CNPointAllo(to.x, -yline);
        }
    }
}

/**
 * Computes the distance to the fringe of the field from a point towards a direction given as angle
 */
double MSLFootballField::distanceToLine(geometry::CNPointAllo from, double angle)
{
    return distanceToLine(from, angle, 0);
}

double MSLFootballField::distanceToLine(geometry::CNPointAllo from, double angle, double extendFieldLines)
{
    double distance = 100000;
    double vx = cos(angle);
    double vy = sin(angle);
    double d;
    double x = FieldLength / 2 + extendFieldLines;
    double y = FieldWidth / 2 + extendFieldLines;
    // Left Line: (1,0)
    d = (y - from.y) / vy;
    if (d > 0)
        distance = min(distance, d);
    // Right Line: (1,0)
    d = (-y - from.y) / vy;
    if (d > 0)
        distance = min(distance, d);
    // Top Line: (0,1)
    d = (x - from.x) / vx;
    if (d > 0)
        distance = min(distance, d);
    // Bot Line: (0,1)
    d = (-x - from.x) / vx;
    if (d > 0)
        distance = min(distance, d);
    return distance;
}

double MSLFootballField::projectVectorOntoX(geometry::CNPointAllo origin, geometry::CNPointAllo dir, double x)
{
    // origin.x + dir.x *t = x =>
    // t = (x-origin.x)/dir.x;
    // y= origin.y+dir.y*(x-origin.x)/dir.x;
    return origin.y + dir.y * (x - origin.x) / dir.x;
}

bool MSLFootballField::cornerCircleExists()
{
    return CornerCircleExists;
}

double MSLFootballField::getCornerCircleRadius()
{
    return CornerCircleRadius;
}

double MSLFootballField::getFieldLength()
{
    return FieldLength;
}

double MSLFootballField::getFieldWidth()
{
    return FieldWidth;
}

double MSLFootballField::getGoalAreaLength()
{
    return GoalAreaLength;
}

double MSLFootballField::getGoalAreaWidth()
{
    return GoalAreaWidth;
}

bool MSLFootballField::isGoalInnerAreaExists()
{
    return GoalInnerAreaExists;
}

double MSLFootballField::getGoalWidth()
{
    return GoalWidth;
}

double MSLFootballField::getLineWidth()
{
    return LineWidth;
}

double MSLFootballField::getMaxDistance()
{
    return MaxDistance;
}

double MSLFootballField::getMaxDistanceSqr()
{
    return MaxDistanceSqr;
}

double MSLFootballField::getMiddleCircleRadius()
{
    return MiddleCircleRadius;
}

double MSLFootballField::getPenaltyAreaLength()
{
    return PenaltyAreaLength;
}

double MSLFootballField::getPenaltyAreaMappingTolerance()
{
    return PenaltyAreaMappingTolerance;
}

double MSLFootballField::getPenaltyAreaWidth()
{
    return PenaltyAreaWidth;
}

double MSLFootballField::getPenaltySpot()
{
    return PenaltySpot;
}

double MSLFootballField::getSurrounding()
{
    return Surrounding;
}

string MSLFootballField::getCurrentField()
{
    return CurrentField;
}

double MSLFootballField::projectVectorOntoY(geometry::CNPointAllo origin, geometry::CNPointAllo dir, double y)
{
    // origin.y + dir.y *t = y =>
    // t = (y-origin.y)/dir.y;
    // x= origin.x+dir.x*(y-origin.y)/dir.y;
    return origin.x + dir.x * (y - origin.y) / dir.y;
}

geometry::CNPointAllo MSLFootballField::posCenterMarker()
{
    return geometry::CNPointAllo(0, 0);
}

geometry::CNPointAllo MSLFootballField::posLeftOwnCorner()
{
    return geometry::CNPointAllo(-FieldLength / 2, FieldWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posRightOwnCorner()
{
    return geometry::CNPointAllo(-FieldLength / 2, -FieldWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posLeftOppCorner()
{
    return geometry::CNPointAllo(FieldLength / 2, FieldWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posRightOppCorner()
{
    return geometry::CNPointAllo(FieldLength / 2, -FieldWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posLeftOwnGoalPost()
{
    return geometry::CNPointAllo(-FieldLength / 2, GoalWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posRightOwnGoalPost()
{
    return geometry::CNPointAllo(-FieldLength / 2, -GoalWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posLeftOppGoalPost()
{
    return geometry::CNPointAllo(FieldLength / 2, GoalWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posRightOppGoalPost()
{
    return geometry::CNPointAllo(FieldLength / 2, -GoalWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posOwnPenaltyMarker()
{
    return geometry::CNPointAllo(-FieldLength / 2 + PenaltySpot, 0.0);
}

geometry::CNPointAllo MSLFootballField::posRightOwnRestartMarker()
{
    return geometry::CNPointAllo(-FieldLength / 2 + PenaltySpot, -FieldWidth / 4);
}

geometry::CNPointAllo MSLFootballField::posLeftOwnRestartMarker()
{
    return geometry::CNPointAllo(-FieldLength / 2 + PenaltySpot, FieldWidth / 4);
}

geometry::CNPointAllo MSLFootballField::posOppPenaltyMarker()
{
    return geometry::CNPointAllo(FieldLength / 2 - PenaltySpot, 0.0);
}

geometry::CNPointAllo MSLFootballField::posRightOppRestartMarker()
{
    return geometry::CNPointAllo(FieldLength / 2 - PenaltySpot, -FieldWidth / 4);
}

geometry::CNPointAllo MSLFootballField::posLeftOppRestartMarker()
{
    return geometry::CNPointAllo(FieldLength / 2 - PenaltySpot, FieldWidth / 4);
}

geometry::CNPointAllo MSLFootballField::posOppGoalMid()
{
    return geometry::CNPointAllo(FieldLength / 2, 0);
}

geometry::CNPointAllo MSLFootballField::posOwnGoalMid()
{
    return geometry::CNPointAllo(-FieldLength / 2, 0);
}

geometry::CNPointAllo MSLFootballField::posLROppHalf()
{
    return geometry::CNPointAllo(0.0, -FieldWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posULOwnHalf()
{
    return geometry::CNPointAllo(0.0, FieldWidth / 2);
}

// TODO calculate penalty stuff with right parameters from globals.conf
geometry::CNPointAllo MSLFootballField::posLROwnPenaltyArea()
{
    return geometry::CNPointAllo(-FieldLength / 2, -PenaltyAreaWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posULOwnPenaltyArea()
{
    return geometry::CNPointAllo(-FieldLength / 2 + PenaltyAreaLength, PenaltyAreaWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posLROppPenaltyArea()
{
    return geometry::CNPointAllo(FieldLength / 2 - PenaltyAreaLength, -PenaltyAreaWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posULOppPenaltyArea()
{
    return geometry::CNPointAllo(FieldLength / 2, PenaltyAreaWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posLROwnGoalArea()
{
    return geometry::CNPointAllo(-FieldLength / 2, -GoalAreaWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posULOwnGoalArea()
{
    return geometry::CNPointAllo(-FieldLength / 2 + GoalAreaLength, GoalAreaWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posLROppGoalArea()
{
    return geometry::CNPointAllo(FieldLength / 2 - GoalAreaLength, -GoalAreaWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posULOppGoalArea()
{
    return geometry::CNPointAllo(FieldLength / 2, GoalAreaWidth / 2);
}

geometry::CNPointAllo MSLFootballField::posLeftRestartMarker()
{
    return geometry::CNPointAllo(0.0, FieldWidth / 4);
}

geometry::CNPointAllo MSLFootballField::posRightRestartMarker()
{
    return geometry::CNPointAllo(0.0, -FieldWidth / 4);
}

geometry::CNPointAllo MSLFootballField::posLRSurrounding()
{
    return geometry::CNPointAllo(-FieldLength / 2 - Surrounding, -FieldWidth / 2 + Surrounding);
}

geometry::CNPointAllo MSLFootballField::posULSurrounding()
{
    return geometry::CNPointAllo(FieldLength / 2 + Surrounding, FieldWidth / 2 - Surrounding);
}
}
