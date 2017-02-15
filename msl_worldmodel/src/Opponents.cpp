#include "Opponents.h"
#include "Ball.h"

#include "MSLWorldModel.h"
#include "Robots.h"

#include <cnc_geometry/Calculator.h>
#include <math.h>

namespace msl
{

Opponents::Opponents(MSLWorldModel *wm, int ringBufferLength)
    : opponentsAlloClustered(ringBufferLength)
    , opponentsEgoClustered(ringBufferLength)
{
    this->wm = wm;
    this->sc = supplementary::SystemConfig::getInstance();
    this->opponentProtectAngle = (*sc)["Dribble"]->get<double>("Dribble.OpponentProtectAngle", NULL);
    this->opponentProtectDistance = (*sc)["Dribble"]->get<double>("Dribble.OpponentProtectDistance", NULL);
}

Opponents::~Opponents()
{
}

std::shared_ptr<geometry::CNPointEgo> Opponents::getInCorridor(double angle, double width)
{
    auto opps = wm->robots->opponents.getOpponentsEgoClustered();
    if (opps == nullptr)
        return nullptr;
    std::shared_ptr<geometry::CNPointEgo> closest;
    double dist = std::numeric_limits<double>::max();
    double temp;
    for (int i = 0; i < opps->size(); i++)
    {
        temp = opps->at(i)->length();
        if (temp < dist)
        {
            double dang = geometry::deltaAngle(angle, opps->at(i)->angleZ());
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
shared_ptr<geometry::CNPointEgo> Opponents::getClosestToBall(double &distance)
{
    distance = std::numeric_limits<double>::max();
    auto obs = getOpponentsEgoClustered();
    if (obs == nullptr)
        return nullptr;
    auto ball = wm->ball->getEgoBallPosition();
    if (ball == nullptr)
        return nullptr;
    double temp;
    shared_ptr<geometry::CNPointEgo> res;
    for (int i = 0; i < obs->size(); i++)
    {
        temp = obs->at(i)->distanceTo(*ball);
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

const InfoBuffer<vector<shared_ptr<geometry::CNPointAllo>>> &Opponents::getOpponentsAlloClusteredBuffer() const
{
    return this->opponentsAlloClustered;
}

void Opponents::processOpponentsAlloClustered(
    shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>> opponentsAlloClustered)
{
    auto o = std::make_shared<InformationElement<std::vector<std::shared_ptr<geometry::CNPointAllo>>>>(
        opponentsAlloClustered, wm->getTime(), this->maxValidity, 1);

    this->opponentsAlloClustered.add(o);
}

const InfoBuffer<vector<shared_ptr<geometry::CNPointAllo>>> &Opponents::getOpponentsEgoClusteredBuffer() const
{
    return this->opponentsEgoClustered;
}

void Opponents::processOpponentsEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPointEgo>>> opponentsEgoClustered)
{
    auto o = make_shared<InformationElement<vector<shared_ptr<geometry::CNPointEgo>>>>(
        opponentsEgoClustered, wm->getTime(), this->maxValidity, 1);

    this->opponentsEgoClustered.add(o);
}

} /* namespace msl */
