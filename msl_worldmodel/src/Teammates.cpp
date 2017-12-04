#include "Teammates.h"
#include "MSLFootballField.h"
#include "MSLWorldModel.h"

#include <cnc_geometry/CNPositionAllo.h>

#include <msl_sensor_msgs/SharedWorldInfo.h>
#include <nonstd/optional.hpp>

using supplementary::InfoBuffer;
using supplementary::InformationElement;
using supplementary::InfoTime;

namespace msl
{
using nonstd::optional;
using std::vector;
using std::shared_ptr;
using std::make_shared;

Teammates::Teammates(MSLWorldModel *wm, int ringBufferLength)
    : teammatesEgoClustered(ringBufferLength)
    , teammatesAlloClustered(ringBufferLength)
{
    this->wm = wm;
}

Teammates::~Teammates()
{
}

/**
 * Returns the number of teammates in the own penalty area.
 * @return Number of teammates in own penalty area.
 */
int Teammates::teammatesInOwnPenalty()
{
    auto teamMatePositions = getPositionsOfTeamMates();
    if (!teamMatePositions)
    {
        return 0;
    }

    int count = 0;
    int myId = wm->getOwnId();

    for (unsigned long i = 0; i < teamMatePositions->size(); i++)
    {
        if (teamMatePositions->at(i).first != myId)
        {
            if (wm->field->isInsideOwnPenalty(teamMatePositions->at(i).second.getPoint(), 0.0))
            {
                count++;
            }
        }
    }
    return count;
}

/**
 * Returns the number of teammates in the opponents penalty area.
 * @return Number of teammates in opp penalty area.
 */
int Teammates::teammatesInOppPenalty()
{
    auto teamMatePositions = getPositionsOfTeamMates();
    if (!teamMatePositions)
    {
        return 0;
    }

    int count = 0;
    int myId = wm->getOwnId();

    for (unsigned long i = 0; i < teamMatePositions->size(); i++)
    {
        if (teamMatePositions->at(i).first != myId)
        {
            if (wm->field->isInsideOppPenalty(teamMatePositions->at(i).second.getPoint(), 100.0))
            {
                count++;
            }
        }
    }
    return count;
}

const InfoBuffer<geometry::CNPositionAllo> &Teammates::getTeammatePositionBuffer(int teammateId)
{
    // create buffer if not present
    auto robotPositionsIter = this->robotPositions.find(teammateId);
    if (robotPositionsIter == this->robotPositions.end())
    {
        auto buffer = std::make_shared<InfoBuffer<geometry::CNPositionAllo>>(wm->getRingBufferLength());
        this->robotPositions.emplace(teammateId, buffer);
        return *buffer;
    }
    else
    {
        return *robotPositionsIter->second;
    }
}

shared_ptr<vector<pair<int, geometry::CNPositionAllo>>> Teammates::getPositionsOfTeamMates()
{
    auto ret = make_shared<vector<pair<int, geometry::CNPositionAllo>>>();
    for (auto iter = robotPositions.begin(); iter != robotPositions.end(); iter++)
    {
        if (iter->second->getLast() != nullptr &&
            wm->getTime() - iter->second->getLast()->getCreationTime() < this->maxValidity)
        {
            auto element = pair<int, geometry::CNPositionAllo>(iter->first, iter->second->getLast()->getInformation());
            ret->push_back(element);
        }
    }
    return ret;
}

const InfoBuffer<std::vector<geometry::CNPointAllo>> &Teammates::getTeammatesAlloClusteredBuffer()
{
    return this->teammatesAlloClustered;
}

void msl::Teammates::integrateTeammatesAlloClustered(shared_ptr<const vector<geometry::CNPointAllo>> teammatesAlloClustered)
{
    auto infoElement = make_shared<InformationElement<vector<geometry::CNPointAllo>>>(
        *teammatesAlloClustered, this->wm->getTime(), this->maxValidity, 1.0);

    this->teammatesAlloClustered.add(infoElement);
}

const InfoBuffer<vector<geometry::CNPointEgo>> &Teammates::getTeammatesEgoClusteredBuffer()
{
    return this->teammatesEgoClustered;
}

void Teammates::integrateTeammatesEgoClustered(shared_ptr<const vector<geometry::CNPointEgo>> teammatesEgoClustered)
{
    auto infoElement = make_shared<InformationElement<vector<geometry::CNPointEgo>>>(
        *teammatesEgoClustered, wm->getTime(), this->maxValidity, 1.0);

    this->teammatesEgoClustered.add(infoElement);
}

void Teammates::integrateTeammatesPosition(msl_sensor_msgs::SharedWorldInfoPtr msg, InfoTime creationTime)
{
    auto sharedCNPositionAllo = geometry::CNPositionAllo(msg->odom.position.x, msg->odom.position.y, msg->odom.position.angle);

    auto infoElement = make_shared<InformationElement<geometry::CNPositionAllo>>(
        sharedCNPositionAllo, creationTime, this->maxValidity, msg->odom.position.certainty);

    auto robotPositionBufferIter = this->robotPositions.find(msg->senderID);
    if (robotPositionBufferIter == this->robotPositions.end())
    {
        auto buffer = make_shared<InfoBuffer<geometry::CNPositionAllo>>(wm->getRingBufferLength());
        this->robotPositions.emplace(msg->senderID, buffer);
        buffer->add(infoElement);
    }
    else
    {
        robotPositionBufferIter->second->add(infoElement);
    }
}

} /* namespace msl */
