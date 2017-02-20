/*
 * Opponents.cpp
 *
 *  Created on: Feb 26, 2016
 *      Author: Stefan Jakob
 */

#include "Opponents.h"
#include "Ball.h"
#include "GeometryCalculator.h"
#include "MSLWorldModel.h"
#include "Robots.h"

namespace msl
{

Opponents::Opponents(MSLWorldModel *wm, int ringBufferLength)
    : opponentsAlloClustered(ringBufferLength)
    , opponentsEgoClustered(ringBufferLength)
{
    this->wm = wm;
    this->sc = supplementary::SystemConfig::getInstance();
    this->ringBufferLength = ringBufferLength;
    this->opponentProtectAngle = (*sc)["Dribble"]->get<double>("Dribble.OpponentProtectAngle", NULL);
    this->opponentProtectDistance = (*sc)["Dribble"]->get<double>("Dribble.OpponentProtectDistance", NULL);
}

Opponents::~Opponents()
{
}

shared_ptr<geometry::CNPoint2D> Opponents::getInCorridor(double angle, double width)
{
    auto opps = wm->robots->opponents.getOpponentsEgoClustered();
    // wm.GetCurrentOpponentListMerged();
    if (opps == nullptr)
        return nullptr;
    shared_ptr<geometry::CNPoint2D> closest;
    double dist = std::numeric_limits<double>::max();
    double temp;
    for (int i = 0; i < opps->size(); i++)
    {
        temp = opps->at(i)->length();
        if (temp < dist)
        {
            double dang = geometry::deltaAngle(angle, opps->at(i)->angleTo());
            if (abs(dang) < M_PI / 2.0)
            {
                if (sin(dang) * temp < width + 300)
                { // 300 = robotradius
                    dist = temp;
                    closest = opps->at(i);
                }
            }
        }
    }
    return closest;
}
shared_ptr<geometry::CNPoint2D> Opponents::getClosestToBall(double &distance)
{
    distance = std::numeric_limits<double>::max();
    auto obs = getOpponentsEgoClustered();
    // wm.GetCurrentOpponentListMerged();
    if (obs == nullptr)
        return nullptr;
    auto ball = wm->ball->getEgoBallPosition();
    if (ball == nullptr)
        return nullptr;
    double temp;
    shared_ptr<geometry::CNPoint2D> res;
    for (int i = 0; i < obs->size(); i++)
    {
        temp = obs->at(i)->distanceTo(ball);
        if (temp < distance)
        {
            distance = temp;
            res = obs->at(i);
        }
    }
    return res;
}
double Opponents::getOpponentProtectDistance()
{
    return this->opponentProtectDistance;
}

double Opponents::getOpponentProtectAngle()
{
    return this->opponentProtectAngle;
}

shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> Opponents::getOpponentsAlloClustered(int index)
{
    auto x = opponentsAlloClustered.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}

void Opponents::processOpponentsAlloClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponentsAlloClustered)
{
    shared_ptr<InformationElement<vector<shared_ptr<geometry::CNPoint2D>>>> o =
        make_shared<InformationElement<vector<shared_ptr<geometry::CNPoint2D>>>>(opponentsAlloClustered, wm->getTime());
    o->certainty = 1;

    this->opponentsAlloClustered.add(o);
}

shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> Opponents::getOpponentsEgoClustered(int index)
{
    auto x = opponentsEgoClustered.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}

void Opponents::processOpponentsEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponentsEgoClustered)
{
    shared_ptr<InformationElement<vector<shared_ptr<geometry::CNPoint2D>>>> o =
        make_shared<InformationElement<vector<shared_ptr<geometry::CNPoint2D>>>>(opponentsEgoClustered, wm->getTime());
    o->certainty = 1;

    this->opponentsEgoClustered.add(o);
}

} /* namespace msl */
