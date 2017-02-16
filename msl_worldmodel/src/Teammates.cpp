#include "Teammates.h"
#include "MSLFootballField.h"
#include "MSLWorldModel.h"

#include <cnc_geometry/CNPositionAllo.h>

#include <msl_sensor_msgs/SharedWorldInfo.h>
#include <nonstd/optional.hpp>

namespace msl
{
using nonstd::optional;

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
    int count = 0;
    int myId = wm->getOwnId();
    shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPositionAllo>>>>> teamMatePositions =
        getPositionsOfTeamMates();
    for (int i = 0; i < teamMatePositions->size(); i++)
    {
        if (teamMatePositions->at(i)->first != myId)
        {
            if (wm->field->isInsideOwnPenalty(teamMatePositions->at(i)->second->getPoint(), 0.0))
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
    int count = 0;
    int myId = wm->getOwnId();
    shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPositionAllo>>>>> teamMatePositions =
        getPositionsOfTeamMates();
    for (int i = 0; i < teamMatePositions->size(); i++)
    {
        if (teamMatePositions->at(i)->first != myId)
        {
            if (wm->field->isInsideOppPenalty(teamMatePositions->at(i)->second->getPoint(), 100.0))
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

shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPositionAllo>>>>> Teammates::getPositionsOfTeamMates()
{
    shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPositionAllo>>>>> ret =
        make_shared<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPositionAllo>>>>>();
    for (auto iter = robotPositions.begin(); iter != robotPositions.end(); iter++)
    {
        if (wm->getTime() - iter->second->getLast()->getCreationTime() < this->maxValidity)
        {
            shared_ptr<pair<int, shared_ptr<geometry::CNPositionAllo>>> element =
                make_shared<pair<int, shared_ptr<geometry::CNPositionAllo>>>(iter->first,
                                                                             iter->second->getLast()->getInformation());
            ret->push_back(element);
        }
    }
    return ret;
}

shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>> Teammates::getTeammatesAlloClustered(int index)
{
    auto x = teammatesAlloClustered.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}

void msl::Teammates::integrateTeammatesAlloClustered(
    shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>> teammatesAlloClustered)
{
    auto infoElement = make_shared<InformationElement<vector<shared_ptr<geometry::CNPointAllo>>>>(
        teammatesAlloClustered, this->wm->getTime(), this->maxValidity, 1.0);

    this->teammatesAlloClustered.add(infoElement);
}

shared_ptr<vector<shared_ptr<geometry::CNPointEgo>>> Teammates::getTeammatesEgoClustered(int index)
{
    auto x = teammatesEgoClustered.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}

void Teammates::processTeammatesEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPointEgo>>> teammatesEgoClustered)
{
    shared_ptr<InformationElement<vector<shared_ptr<geometry::CNPointEgo>>>> o =
        make_shared<InformationElement<vector<shared_ptr<geometry::CNPointEgo>>>>(teammatesEgoClustered, wm->getTime());
    o->certainty = 1;

    this->teammatesEgoClustered.add(o);
}

void Teammates::integrateTeammatesPosition(msl_sensor_msgs::SharedWorldInfoPtr msg, InfoTime creationTime)
{
    auto sharedCNPositionAllo =
        make_shared<geometry::CNPositionAllo>(msg->odom.position.x, msg->odom.position.y, msg->odom.position.angle);

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
