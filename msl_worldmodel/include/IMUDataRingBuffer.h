#ifndef AVERAGE_RING_BUFFER_H_
#define AVERAGE_RING_BUFFER_H_
#include "InfoBuffer.h"
#include "msl_actuator_msgs/IMUData.h"
#include "InformationElement.h"

namespace msl {
	class IMUDataRingBuffer : public InfoBuffer<msl_actuator_msgs::IMUData> {
	public:
		IMUDataRingBuffer(const int x) : InfoBuffer(x) {

		}

		double getAverageMod() {
			double x = 0, y = 0;
			for (int i = 0; i <this->getSize();i++) {
				msl_msgs::Point3dInfo_<std::allocator<void> > magnet = this->getLast(i)->getInformation().magnet;
				double bearing = atan2(magnet.y,magnet.x);
				x+= cos(bearing);
				y+= sin(bearing);
			}
			return atan2(y,x);
		}
	};
}
#endif
