#include "Opponents.h"
#include "Ball.h"

#include "MSLWorldModel.h"
#include "Robots.h"

#include <cnc_geometry/Calculator.h>
#include <math.h>

namespace msl
{

using std::vector;
using std::shared_ptr;
using std::make_shared;

using nonstd::optional;
using nonstd::nullopt;
using nonstd::make_optional;

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

optional<geometry::CNPointEgo> Opponents::getInCorridor(double angle, double width)
{
    auto opps = wm->robots->opponents.getOpponentsEgoClustered();
    if (opps == nullptr)
        return nullptr;

    bool found = false;
    geometry::CNPointEgo closest;
    double bestDist = std::numeric_limits<double>::max();
    double temp;
    for (unsigned long i = 0; i < opps->size(); i++)
    {
        temp = opps->at(i).length();
        if (temp < bestDist)
        {
            double dang = geometry::deltaAngle(angle, opps->at(i).angleZ());
            if (abs(dang) < M_PI / 2.0)
            {
                if (sin(dang) * temp < width + 300)
                { // 300 = robotradius
                    bestDist = temp;
                    closest = opps->at(i);
                    found = true;
                }
            }
        }
    }

    if (found)
    {
        return make_optional<geometry::CNPointEgo>(closest);
    }
    return nullopt;
}

optional<geometry::CNPointEgo> Opponents::getClosestToBall(double &distance)
{
    distance = std::numeric_limits<double>::max();
    auto obs = getOpponentsEgoClustered();
    if (obs == nullptr)
        return nullopt;
    auto ball = wm->ball->getEgoBallPosition();
    if (ball == nullptr)
        return nullopt;

    bool found = false;
    geometry::CNPointEgo closest;
    for (int i = 0; i < obs->size(); i++)
    {
        double curDist = obs->at(i).distanceTo(*ball);
        if (curDist < distance)
        {
            distance = curDist;
            closest = obs->at(i);
            found = true;
        }
    }
    if (found)
    {
        return make_optional<geometry::CNPointEgo>(closest);
    }

    return nullopt;
}

double Opponents::getOpponentProtectDistance()
{
    return this->opponentProtectDistance;
}

double Opponents::getOpponentProtectAngle()
{
    return this->opponentProtectAngle;
}

const InfoBuffer<vector<geometry::CNPointAllo>> &Opponents::getOpponentsAlloClusteredBuffer() const
{
    return this->opponentsAlloClustered;
}

void Opponents::processOpponentsAlloClustered(shared_ptr<vector<geometry::CNPointAllo>> opponentsAlloClustered)
{
    auto o = make_shared<InformationElement<vector<geometry::CNPointAllo>>>(opponentsAlloClustered, wm->getTime(),
                                                                            this->maxValidity, 1);

    this->opponentsAlloClustered.add(o);
}

const InfoBuffer<vector<geometry::CNPointEgo>> &Opponents::getOpponentsEgoClusteredBuffer() const
{
    return this->opponentsEgoClustered;
}

void Opponents::processOpponentsEgoClustered(shared_ptr<vector<geometry::CNPointEgo>> opponentsEgoClustered)
{
    auto o = make_shared<InformationElement<vector<geometry::CNPointEgo>>>(opponentsEgoClustered, wm->getTime(),
                                                                           this->maxValidity, 1);

    this->opponentsEgoClustered.add(o);
}

} /* namespace msl */
