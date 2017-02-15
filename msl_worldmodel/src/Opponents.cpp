#include "Opponents.h"
#include "Ball.h"

#include "MSLWorldModel.h"
#include "Robots.h"

#include <cnc_geometry/Calculator.h>

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

shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>> Opponents::getOpponentsAlloClustered(int index)
{
    auto x = opponentsAlloClustered.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}

void Opponents::processOpponentsAlloClustered(shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>> opponentsAlloClustered)
{
    shared_ptr<InformationElement<vector<shared_ptr<geometry::CNPointAllo>>>> o =
        make_shared<InformationElement<vector<shared_ptr<geometry::CNPointAllo>>>>(opponentsAlloClustered, wm->getTime());
    o->certainty = 1;

    this->opponentsAlloClustered.add(o);
}

shared_ptr<vector<shared_ptr<geometry::CNPointEgo>>> Opponents::getOpponentsEgoClustered(int index)
{
    auto x = opponentsEgoClustered.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}

void Opponents::processOpponentsEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPointEgo>>> opponentsEgoClustered)
{
    shared_ptr<InformationElement<vector<shared_ptr<geometry::CNPointEgo>>>> o =
        make_shared<InformationElement<vector<shared_ptr<geometry::CNPointEgo>>>>(opponentsEgoClustered, wm->getTime());
    o->certainty = 1;

    this->opponentsEgoClustered.add(o);
}

} /* namespace msl */
