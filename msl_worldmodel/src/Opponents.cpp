#include "Opponents.h"
#include "Ball.h"

#include "MSLWorldModel.h"
#include "Robots.h"

#include <cnc_geometry/Calculator.h>
#include <math.h>

using supplementary::InformationElement;
using supplementary::InfoBuffer;

namespace msl
{

using std::vector;
using std::shared_ptr;
using std::make_shared;

using nonstd::optional;
using nonstd::nullopt;

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
    auto oppsInfo = this->getOpponentsEgoClusteredBuffer().getLastValid();
    if (oppsInfo == nullptr)
        return nullopt;

    auto opps = oppsInfo->getInformation();

    bool found = false;
    geometry::CNPointEgo closest;
    double bestDist = std::numeric_limits<double>::max();
    double temp;

    for (unsigned long i = 0; i < opps.size(); i++)
    {
        temp = opps.at(i).length();
        if (temp < bestDist)
        {
            double dang = geometry::deltaAngle(angle, opps.at(i).angleZ());
            if (abs(dang) < M_PI / 2.0)
            {
                if (sin(dang) * temp < width + 300)
                { // 300 = robotradius
                    bestDist = temp;
                    closest = opps.at(i);
                    found = true;
                }
            }
        }
    }

    if (found)
    {
        return closest;
    }
    return nullopt;
}

optional<geometry::CNPointEgo> Opponents::getClosestToBall(double &distance)
{
    auto oppsInfo = this->getOpponentsEgoClusteredBuffer().getLastValid();
    if (oppsInfo == nullptr)
        return nullopt;

    auto opps = oppsInfo->getInformation();

    auto ball = wm->ball->getPositionEgo();
    if (ball)
        return nullopt;

    bool found = false;
    distance = std::numeric_limits<double>::max();
    geometry::CNPointEgo closest;

    for (unsigned long i = 0; i < opps.size(); i++)
    {
        double curDist = opps.at(i).distanceTo(*ball);
        if (curDist < distance)
        {
            distance = curDist;
            closest = opps.at(i);
            found = true;
        }
    }

    if (found)
    {
        return closest;
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

void Opponents::integrateOpponentsAlloClustered(shared_ptr<const vector<geometry::CNPointAllo>> opponentsAlloClustered)
{
    auto o = make_shared<InformationElement<vector<geometry::CNPointAllo>>>(*opponentsAlloClustered, wm->getTime(),
                                                                            this->maxValidity, 1.0);

    this->opponentsAlloClustered.add(o);
}

const InfoBuffer<vector<geometry::CNPointEgo>> &Opponents::getOpponentsEgoClusteredBuffer() const
{
    return this->opponentsEgoClustered;
}

void Opponents::integrateOpponentsEgoClustered(shared_ptr<const vector<geometry::CNPointEgo>> opponentsEgoClustered)
{
    auto o = make_shared<InformationElement<vector<geometry::CNPointEgo>>>(*opponentsEgoClustered, wm->getTime(),
                                                                           this->maxValidity, 1.0);

    this->opponentsEgoClustered.add(o);
}

} /* namespace msl */
