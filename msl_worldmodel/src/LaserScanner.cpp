#include "LaserScanner.h"

#include "MSLWorldModel.h"

namespace msl {
LaserScanner::LaserScanner(MSLWorldModel* wm, int ringBufferLength) : goalWallPositions(ringBufferLength)
{
	this->wm = wm;
	this->ringBufferlength = ringBufferLength;
}

LaserScanner::~LaserScanner()
{
    // TODO Auto-generated destructor stub
}

void LaserScanner::processLaserScannPoints(msl_sensor_msgs::LaserLocalizationPtr msg)
{
	shared_ptr<msl_sensor_msgs::LaserLocalization> laserScan =
	            shared_ptr<msl_sensor_msgs::LaserLocalization>(msg.get(), [msg](msl_sensor_msgs::LaserLocalization *) mutable { msg.reset(); });
    shared_ptr<InformationElement<msl_sensor_msgs::LaserLocalization>> scan = make_shared<InformationElement<msl_sensor_msgs::LaserLocalization>>(laserScan, wm->getTime());
    scan->certainty = 1.0;
    goalWallPositions.add(scan);
}

shared_ptr<msl_sensor_msgs::LaserLocalization> LaserScanner::getGoalWallPosition(int index)
{
    auto x = goalWallPositions.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
    {
        return nullptr;
    }
    return x->getInformation();
}
}
