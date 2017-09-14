#ifndef AVERAGE_RING_BUFFER_H_
#define AVERAGE_RING_BUFFER_H_
#include "RingBuffer.h"
#include "msl_actuator_msgs/IMUData.h"
#include "InformationElement.h"

namespace msl {
	class IMUDataRingBuffer : public RingBuffer<InformationElement<msl_actuator_msgs::IMUData>> {
	public:
		IMUDataRingBuffer(const int x) : RingBuffer(x) {

		}

		double getAverageMod() {
			double x = 0, y = 0;
			for (int i = 0; i <this->getSize();i++) {
				msl_msgs::Point3dInfo_<std::allocator<void> > magnet = this->getElement(i)->getInformation()->magnet;
				double bearing = atan2(magnet.y,magnet.x);
				x+= cos(bearing);
				y+= sin(bearing);
			}
			return atan2(y,x);
		}
	};
}
#endif
