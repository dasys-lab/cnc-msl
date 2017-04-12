/*
 * BallXIntervall.cpp
 *
 *  Created on: May 2, 2016
 *      Author: Stefan Jakob
 */

#include <Ball.h>
#include <BallXIntervall.h>
#include <MSLWorldModel.h>

using std::vector;
using nonstd::nullopt;

namespace alica
{

BallXIntervall::BallXIntervall(double weight, string name, long id, vector<long> relevantEntryPointIds, double minX,
                               double maxX, double tolerance)
{
    this->weight = weight;
    this->name = name;
    this->id = id;
    this->minX = minX;
    this->maxX = maxX;
    this->alloBall = nullopt;
    this->relevantEntryPointIds = relevantEntryPointIds;
    this->tolerance = tolerance;
}

BallXIntervall::~BallXIntervall()
{
}

void BallXIntervall::cacheEvalData()
{
    this->alloBall = msl::MSLWorldModel::get()->ball->getPositionAllo();
}

UtilityInterval BallXIntervall::eval(IAssignment *ass)
{
    this->ui.setMin(0.0);
    this->ui.setMax(0.0);

    if (!alloBall)
    {
        this->ui.setMin(0.0);
        this->ui.setMax(0.0);
        return ui;
    }

    double x = alloBall->x;
    if (x >= this->minX && x <= this->maxX)
    {
        this->ui.setMin(1.0);
        this->ui.setMax(1.0);
        return ui;
    }

    if (x <= (this->minX - this->tolerance))
    {
        this->ui.setMin(-1.0);
        this->ui.setMax(-1.0);
        return ui;
    }

    if (x >= (this->maxX + this->tolerance))
    {
        this->ui.setMin(-1.0);
        this->ui.setMax(-1.0);
        return ui;
    }

    double val = 0.0;
    if (x > (this->minX - this->tolerance) && x < this->minX)
    {
        val = x / this->minX;
    }

    if (x > this->maxX && x < (this->maxX + tolerance))
    {
        val = 1 - ((x - this->maxX) / (this->maxX + tolerance - this->maxX));
    }
    this->ui.setMin(val);
    this->ui.setMax(val);
    return ui;
}

string BallXIntervall::toString()
{
    string retString = this->name + ": ";
    retString += string("W: ") + to_string(this->weight);
    retString += string(" minX: ") + to_string(this->minX);
    retString += string(" maxX: ") + to_string(this->maxX);
    retString += string(" alloBallPos: ") + this->alloBall->toString();
    return retString;
}

} /* namespace alica */
