#include "ExtensionHelper.h"
#include "SpicaHelper.h"
#include "msl_actuator_msgs/KickControl.h"

using namespace msl_actuator_msgs;

#define BUF_SIZE 500

ros::Publisher ExtensionHelper::kcpub;

void ExtensionHelper::initialize() {
	kcpub = SpicaHelper::visionNode->advertise<KickControl>("KickControl", 1);
}

//Not Testet
int ExtensionHelper::extendUpperExtension()
{
		KickControl kick;
		kick.extension = (1);
		kick.extTime = (1000);
		kcpub.publish(kick);
}
